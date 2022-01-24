#!/usr/bin/env python3
#
# Copyright (c) 2022 Kaz Kojima <kkojima@rr.iij4u.or.jp>
# SPDX-License-Identifier: BSD--3-Clause

import os

from amaranth import *
from amaranth.lib.cdc    import FFSynchronizer
from amaranth.hdl.rec    import DIR_FANIN, DIR_FANOUT, DIR_NONE
from amaranth.back       import verilog
from amaranth.lib.fifo   import SyncFIFO

from amlib.stream        import StreamInterface, connect_stream_to_fifo, connect_fifo_to_stream
from amlib.utils         import EdgeToPulse, Timer

from luna.usb2           import USBDevice, USBIsochronousInMemoryEndpoint, USBIsochronousOutStreamEndpoint, USBIsochronousInStreamEndpoint

from usb_protocol.types                       import USBRequestType, USBRequestRecipient, USBTransferType, USBSynchronizationType, USBUsageType, USBDirection, USBStandardRequests
from usb_protocol.types.descriptors.uac2      import AudioClassSpecificRequestCodes
from usb_protocol.emitters                    import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors        import uac2, standard

from luna.gateware.platform                   import NullPin
from luna.gateware.usb.usb2.device            import USBDevice
from luna.gateware.usb.usb2.request           import USBRequestHandler, StallOnlyRequestHandler
from luna.gateware.usb.stream                 import USBInStreamInterface
from luna.gateware.stream.generator           import StreamSerializer
from luna.gateware.architecture.car import PHYResetController

from pdm.pdm2pcm import PDM2PCM
from pdm.pcm2pdm import PCM2PDM

from requesthandlers import UAC2RequestHandlers

class LunaUSBAudioDevice(Elaboratable):
    """ USB Audio Class v2 interface """
    NR_CHANNELS = 1
    MAX_PACKET_SIZE = 512 # NR_CHANNELS * 24 + 4
    USE_ILA = False
    ILA_MAX_PACKET_SIZE = 512
    ENABLE_PDM_IN = True
    ENABLE_PDM_OUT = True

    def create_descriptors(self):
        """ Creates the descriptors that describe our audio topology. """

        descriptors = DeviceDescriptorCollection()

        with descriptors.DeviceDescriptor() as d:
            d.bcdUSB             = 2.00
            d.bDeviceClass       = 0xEF
            d.bDeviceSubclass    = 0x02
            d.bDeviceProtocol    = 0x01
            d.idVendor           = 0x1209
            d.idProduct          = 0x5af1

            d.iManufacturer      = "GsD"
            d.iProduct           = "ButterStick r1.0 LunaUAC2"
            d.iSerialNumber      = "4712"
            d.bcdDevice          = 0.01

            d.bNumConfigurations = 1

        with descriptors.ConfigurationDescriptor() as configDescr:
            # Interface Association
            interfaceAssociationDescriptor                 = uac2.InterfaceAssociationDescriptorEmitter()
            interfaceAssociationDescriptor.bInterfaceCount = 3 # Audio Control + Inputs + Outputs
            configDescr.add_subordinate_descriptor(interfaceAssociationDescriptor)

            # Interface Descriptor (Control)
            interfaceDescriptor = uac2.StandardAudioControlInterfaceDescriptorEmitter()
            interfaceDescriptor.bInterfaceNumber = 0
            configDescr.add_subordinate_descriptor(interfaceDescriptor)

            # AudioControl Interface Descriptor
            audioControlInterface = self.create_audio_control_interface_descriptor()
            configDescr.add_subordinate_descriptor(audioControlInterface)

            self.create_output_channels_descriptor(configDescr)

            self.create_input_channels_descriptor(configDescr)

            if self.USE_ILA:
                with configDescr.InterfaceDescriptor() as i:
                    i.bInterfaceNumber = 3

                    with i.EndpointDescriptor() as e:
                        e.bEndpointAddress = USBDirection.IN.to_endpoint_address(3) # EP 3 IN
                        e.wMaxPacketSize   = self.ILA_MAX_PACKET_SIZE

        return descriptors


    def create_audio_control_interface_descriptor(self):
        audioControlInterface = uac2.ClassSpecificAudioControlInterfaceDescriptorEmitter()

        # AudioControl Interface Descriptor (ClockSource)
        clockSource = uac2.ClockSourceDescriptorEmitter()
        clockSource.bClockID     = 1
        clockSource.bmAttributes = uac2.ClockAttributes.INTERNAL_FIXED_CLOCK
        clockSource.bmControls   = uac2.ClockFrequencyControl.HOST_READ_ONLY
        audioControlInterface.add_subordinate_descriptor(clockSource)


        # streaming input port from the host to the USB interface
        inputTerminal               = uac2.InputTerminalDescriptorEmitter()
        inputTerminal.bTerminalID   = 2
        inputTerminal.wTerminalType = uac2.USBTerminalTypes.USB_STREAMING
        # The number of channels needs to be 2 here in order to be recognized
        # default audio out device by Windows. We provide an alternate
        # setting with the full channel count, which also references
        # this terminal ID
        inputTerminal.bNrChannels   = self.NR_CHANNELS
        inputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(inputTerminal)

        # audio output port from the USB interface to the outside world
        outputTerminal               = uac2.OutputTerminalDescriptorEmitter()
        outputTerminal.bTerminalID   = 3
        outputTerminal.wTerminalType = uac2.OutputTerminalTypes.SPEAKER
        outputTerminal.bSourceID     = 2
        outputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(outputTerminal)

        # audio input port from the outside world to the USB interface
        inputTerminal               = uac2.InputTerminalDescriptorEmitter()
        inputTerminal.bTerminalID   = 4
        inputTerminal.wTerminalType = uac2.InputTerminalTypes.MICROPHONE
        inputTerminal.bNrChannels   = self.NR_CHANNELS
        inputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(inputTerminal)

        # audio output port from the USB interface to the host
        outputTerminal               = uac2.OutputTerminalDescriptorEmitter()
        outputTerminal.bTerminalID   = 5
        outputTerminal.wTerminalType = uac2.USBTerminalTypes.USB_STREAMING
        outputTerminal.bSourceID     = 4
        outputTerminal.bCSourceID    = 1
        audioControlInterface.add_subordinate_descriptor(outputTerminal)

        return audioControlInterface


    def create_output_streaming_interface(self, c, *, nr_channels, alt_setting_nr):
        # Interface Descriptor (Streaming, OUT, active setting)
        activeAudioStreamingInterface                   = uac2.AudioStreamingInterfaceDescriptorEmitter()
        activeAudioStreamingInterface.bInterfaceNumber  = 1
        activeAudioStreamingInterface.bAlternateSetting = alt_setting_nr
        activeAudioStreamingInterface.bNumEndpoints     = 2
        c.add_subordinate_descriptor(activeAudioStreamingInterface)

        # AudioStreaming Interface Descriptor (General)
        audioStreamingInterface               = uac2.ClassSpecificAudioStreamingInterfaceDescriptorEmitter()
        audioStreamingInterface.bTerminalLink = 2
        audioStreamingInterface.bFormatType   = uac2.FormatTypes.FORMAT_TYPE_I
        audioStreamingInterface.bmFormats     = uac2.TypeIFormats.PCM
        audioStreamingInterface.bNrChannels   = nr_channels
        c.add_subordinate_descriptor(audioStreamingInterface)

        # AudioStreaming Interface Descriptor (Type I)
        typeIStreamingInterface  = uac2.TypeIFormatTypeDescriptorEmitter()
        typeIStreamingInterface.bSubslotSize   = 2
        typeIStreamingInterface.bBitResolution = 16 # we use all 16 bits
        c.add_subordinate_descriptor(typeIStreamingInterface)

        # Endpoint Descriptor (Audio out)
        audioOutEndpoint = standard.EndpointDescriptorEmitter()
        audioOutEndpoint.bEndpointAddress     = USBDirection.OUT.to_endpoint_address(1) # EP 1 OUT
        audioOutEndpoint.bmAttributes         = USBTransferType.ISOCHRONOUS  | \
                                                (USBSynchronizationType.ASYNC << 2) | \
                                                (USBUsageType.DATA << 4)
        audioOutEndpoint.wMaxPacketSize = self.MAX_PACKET_SIZE
        audioOutEndpoint.bInterval       = 1
        c.add_subordinate_descriptor(audioOutEndpoint)

        # AudioControl Endpoint Descriptor
        audioControlEndpoint = uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptorEmitter()
        c.add_subordinate_descriptor(audioControlEndpoint)

        # Endpoint Descriptor (Feedback IN)
        feedbackInEndpoint = standard.EndpointDescriptorEmitter()
        feedbackInEndpoint.bEndpointAddress  = USBDirection.IN.to_endpoint_address(1) # EP 1 IN
        feedbackInEndpoint.bmAttributes      = USBTransferType.ISOCHRONOUS  | \
                                               (USBSynchronizationType.NONE << 2)  | \
                                               (USBUsageType.FEEDBACK << 4)
        feedbackInEndpoint.wMaxPacketSize    = 4
        feedbackInEndpoint.bInterval         = 4
        c.add_subordinate_descriptor(feedbackInEndpoint)


    def create_output_channels_descriptor(self, c):
        #
        # Interface Descriptor (Streaming, OUT, quiet setting)
        #
        quietAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        quietAudioStreamingInterface.bInterfaceNumber  = 1
        quietAudioStreamingInterface.bAlternateSetting = 0
        c.add_subordinate_descriptor(quietAudioStreamingInterface)

        # we need the default alternate setting to be stereo
        # out for windows to automatically recognize
        # and use this audio interface
        self.create_output_streaming_interface(c, nr_channels=self.NR_CHANNELS, alt_setting_nr=1)


    def create_input_streaming_interface(self, c, *, nr_channels, alt_setting_nr, channel_config=0):
        # Interface Descriptor (Streaming, IN, active setting)
        activeAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        activeAudioStreamingInterface.bInterfaceNumber  = 2
        activeAudioStreamingInterface.bAlternateSetting = alt_setting_nr
        activeAudioStreamingInterface.bNumEndpoints     = 1
        c.add_subordinate_descriptor(activeAudioStreamingInterface)

        # AudioStreaming Interface Descriptor (General)
        audioStreamingInterface                 = uac2.ClassSpecificAudioStreamingInterfaceDescriptorEmitter()
        audioStreamingInterface.bTerminalLink   = 5
        audioStreamingInterface.bFormatType     = uac2.FormatTypes.FORMAT_TYPE_I
        audioStreamingInterface.bmFormats       = uac2.TypeIFormats.PCM
        audioStreamingInterface.bNrChannels     = nr_channels
        audioStreamingInterface.bmChannelConfig = channel_config
        c.add_subordinate_descriptor(audioStreamingInterface)

        # AudioStreaming Interface Descriptor (Type I)
        typeIStreamingInterface  = uac2.TypeIFormatTypeDescriptorEmitter()
        typeIStreamingInterface.bSubslotSize   = 2
        typeIStreamingInterface.bBitResolution = 16 # we use all 16 bits
        c.add_subordinate_descriptor(typeIStreamingInterface)

        # Endpoint Descriptor (Audio out)
        audioOutEndpoint = standard.EndpointDescriptorEmitter()
        audioOutEndpoint.bEndpointAddress     = USBDirection.IN.to_endpoint_address(2) # EP 2 IN
        audioOutEndpoint.bmAttributes         = USBTransferType.ISOCHRONOUS  | \
                                                (USBSynchronizationType.ASYNC << 2) | \
                                                (USBUsageType.DATA << 4)
        audioOutEndpoint.wMaxPacketSize = self.MAX_PACKET_SIZE
        audioOutEndpoint.bInterval      = 1
        c.add_subordinate_descriptor(audioOutEndpoint)

        # AudioControl Endpoint Descriptor
        audioControlEndpoint = uac2.ClassSpecificAudioStreamingIsochronousAudioDataEndpointDescriptorEmitter()
        c.add_subordinate_descriptor(audioControlEndpoint)


    def create_input_channels_descriptor(self, c):
        #
        # Interface Descriptor (Streaming, IN, quiet setting)
        #
        quietAudioStreamingInterface = uac2.AudioStreamingInterfaceDescriptorEmitter()
        quietAudioStreamingInterface.bInterfaceNumber  = 2
        quietAudioStreamingInterface.bAlternateSetting = 0
        c.add_subordinate_descriptor(quietAudioStreamingInterface)

        # Front center
        self.create_input_streaming_interface(c, nr_channels=self.NR_CHANNELS, alt_setting_nr=1, channel_config=0x4)


    def __init__(self):
        self.ulpi = Record(
            [
                ('data', [('i', 8, DIR_FANIN), ('o', 8, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
                ('clk', [('o', 1, DIR_FANOUT)]),
                ('stp', 1, DIR_FANOUT),
                ('nxt', [('i', 1, DIR_FANIN)]),
                ('dir', [('i', 1, DIR_FANIN)]),
                ('rst', 1, DIR_FANOUT)
            ]
        )

        self.pdmin = Record(
            [
                ('data', [('i', 1, DIR_FANIN)]),
                ('clk', [('o', 1, DIR_FANOUT)]),
            ]
        )

        self.pdmout = Record(
            [
                ('data', [('o', 1, DIR_FANOUT)]),
                ('clk', [('o', 1, DIR_FANOUT)]),
            ]
        )

        self.clk_sync = Signal()
        self.clk_usb = Signal()
        self.rst_sync = Signal()

        self.usb_holdoff  = Signal()
        ...

    def elaborate(self, platform):
        m = Module()

        # Create our clock domains.
        m.domains.sync = ClockDomain()
        m.domains.usb  = ClockDomain()
        m.submodules.usb_reset = controller = PHYResetController(reset_length=40e-3, stop_length=40e-4)
        m.d.comb += [
            ResetSignal("usb")  .eq(controller.phy_reset),
            self.usb_holdoff    .eq(controller.phy_stop)
        ]

        # Attach Clock domains
        m.d.comb += [
            ClockSignal(domain="sync")     .eq(self.clk_sync),
            ClockSignal(domain="usb")      .eq(self.clk_usb),
            ResetSignal("sync").eq(self.rst_sync),
        ]

        # Assume 48kHz sampling rate, 60MHz clk_usb and 48 OSR of PDM module.
        # (60000000/48000)/48 = 26.0416...
        if self.ENABLE_PDM_IN:
            m.submodules.pdm_receiver = pdm_receiver = DomainRenamer("usb")(PDM2PCM(divisor=26))

            m.d.comb += [
                # wire up PDM receiver
                pdm_receiver.pdm_data_in.eq(self.pdmin.data),
                self.pdmin.clk.eq(pdm_receiver.pdm_clock_out),
            ]

        if self.ENABLE_PDM_OUT:
            m.submodules.pdm_transmitter = pdm_transmitter = DomainRenamer("usb")(PCM2PDM(divisor=26))

            m.d.comb += [
                # wire up PDM transmitter
                self.pdmout.data.eq(pdm_transmitter.pdm_data_out),
                self.pdmout.clk.eq(pdm_transmitter.pdm_clock_out),
            ]

        self.usb0 = usb = USBDevice(bus=self.ulpi)

        # Attach usb module
        m.submodules.usb0 = self.usb0

        # Add our standard control endpoint to the device.
        descriptors = self.create_descriptors()
        control_ep = usb.add_control_endpoint()
        control_ep.add_standard_request_handlers(descriptors, blacklist=[
            lambda setup:   (setup.type    == USBRequestType.STANDARD)
                          & (setup.request == USBStandardRequests.SET_INTERFACE)
        ])

        # Attach our class request handlers.
        class_request_handler = UAC2RequestHandlers()
        control_ep.add_request_handler(class_request_handler)

        # Attach class-request handlers that stall any vendor or reserved requests,
        # as we don't have or need any.
        stall_condition = lambda setup : \
            (setup.type == USBRequestType.VENDOR) | \
            (setup.type == USBRequestType.RESERVED)
        control_ep.add_request_handler(StallOnlyRequestHandler(stall_condition))

        ep1_out = USBIsochronousOutStreamEndpoint(
            endpoint_number=1, # EP 1 OUT
            max_packet_size=self.MAX_PACKET_SIZE)
        usb.add_endpoint(ep1_out)

        ep1_in = USBIsochronousInMemoryEndpoint(
            endpoint_number=1, # EP 1 IN
            max_packet_size=4)
        usb.add_endpoint(ep1_in)

        ep2_in = USBIsochronousInStreamEndpoint(
            endpoint_number=2, # EP 2 IN
            max_packet_size=self.MAX_PACKET_SIZE)
        usb.add_endpoint(ep2_in)

        # calculate bytes in frame for audio in
        audio_in_frame_bytes = Signal(range(self.MAX_PACKET_SIZE), reset=12 * self.NR_CHANNELS)
        audio_in_frame_bytes_counting = Signal()

        with m.If(ep1_out.stream.valid & ep1_out.stream.ready):
            with m.If(audio_in_frame_bytes_counting):
                m.d.usb += audio_in_frame_bytes.eq(audio_in_frame_bytes + 1)

            with m.If(ep1_out.stream.first):
                m.d.usb += [
                    audio_in_frame_bytes.eq(1),
                    audio_in_frame_bytes_counting.eq(1),
                ]
            with m.Elif(ep1_out.stream.last):
                m.d.usb += audio_in_frame_bytes_counting.eq(0)

        # Connect our device as a high speed device
        m.d.comb += [
            ep1_in.bytes_in_frame.eq(2),
            # 48000*2*0.125*10^-3 = 12
            ep2_in.bytes_in_frame.eq(12),
            usb.connect          .eq(1),
            usb.full_speed_only  .eq(0),
        ]

        # feedback endpoint
        feedbackValue      = Signal(32, reset=0x60000)
        bitPos             = Signal(5)

        # this tracks the number of audio frames since the last USB frame
        # 12.288MHz / 8kHz = 1536, so we need at least 11 bits = 2048
        # we need to capture 32 micro frames to get to the precision
        # required by the USB standard, so and that is 0xc000, so we
        # need 16 bits here
        audio_clock_counter = Signal(16)
        sof_counter         = Signal(5)

        audio_clock_usb = Signal()
        m.submodules.audio_clock_usb_sync = FFSynchronizer(ClockSignal("audio"), audio_clock_usb, o_domain="usb")
        m.submodules.audio_clock_usb_pulse = audio_clock_usb_pulse = DomainRenamer("usb")(EdgeToPulse())
        audio_clock_tick = Signal()
        m.d.usb += [
            audio_clock_usb_pulse.edge_in.eq(audio_clock_usb),
            audio_clock_tick.eq(audio_clock_usb_pulse.pulse_out),
        ]

        with m.If(audio_clock_tick):
            m.d.usb += audio_clock_counter.eq(audio_clock_counter + 1)

        with m.If(usb.sof_detected):
            m.d.usb += sof_counter.eq(sof_counter + 1)

            # according to USB2 standard chapter 5.12.4.2
            # we need 2**13 / 2**8 = 2**5 = 32 SOF-frames of
            # sample master frequency counter to get enough
            # precision for the sample frequency estimate
            # / 2**8 because the ADAT-clock = 256 times = 2**8
            # the sample frequency and sof_counter is 5 bits
            # so it wraps automatically every 32 SOFs
            with m.If(sof_counter == 0):
                m.d.usb += [
                    feedbackValue.eq((audio_clock_counter + 1) << 3),
                    audio_clock_counter.eq(0),
                ]

        m.d.comb += [
            bitPos.eq(ep1_in.address << 3),
            ep1_in.value.eq(0xff & (feedbackValue >> bitPos)),
        ]

        if self.ENABLE_PDM_IN:
            m.submodules.in_fifo = in_fifo = \
                DomainRenamer("usb")(SyncFIFO(width=8, depth=512))

            m.d.comb += connect_fifo_to_stream(in_fifo, ep2_in.stream)

            # Serializer
            # Convert 16-bit numbers to bytes in the little endian way
            # Assume 24-bit PDM2PCM
            pcm24 = Signal(signed(24))
            le16 = Signal(signed(16))
            b8 = Signal(8)
            with m.If(pdm_receiver.pcm_strobe_out):
                m.d.usb += pcm24.eq(pdm_receiver.pcm_data_out)
            m.d.comb += [
                le16.eq(pcm24 >> 8),
                in_fifo.w_data.eq(b8)
            ]
            m.d.usb += in_fifo.w_en.eq(0)
            with m.FSM(reset="WAIT", domain="usb"):
                with m.State("WAIT"):
                    with m.If(pdm_receiver.pcm_strobe_out):
                        m.next="LO"
                with m.State("LO"):
                    with m.If(in_fifo.w_rdy):
                        m.d.usb += [
                            b8.eq(le16[0:8]),
                            in_fifo.w_en.eq(1)
                        ]
                        m.next="LOACK"
                    with m.Else():
                        m.next="WAIT"
                with m.State("LOACK"):
                    m.next="HI"
                with m.State("HI"):
                    with m.If(in_fifo.w_rdy):
                        m.d.usb += [
                            b8.eq(le16[8:16]),
                            in_fifo.w_en.eq(1)
                        ]
                        m.next="HIACK"
                    with m.Else():
                        m.next="WAIT"
                with m.State("HIACK"):
                    m.next="WAIT"

        if self.ENABLE_PDM_OUT:
            m.submodules.out_fifo = out_fifo = \
                DomainRenamer("usb")(SyncFIFO(width=9, depth=512))

            m.d.comb += connect_stream_to_fifo(ep1_out.stream, out_fifo, firstBit=8)

            # Deserializer
            # Convert bytes to signed 16-bit numbers in the little endian way
            se16 = Signal(signed(16))
            sample = Signal(8)
            fbit = Signal()
            m.d.comb += [
                sample.eq(out_fifo.r_data),
                fbit.eq(out_fifo.r_data[8])
            ]
            m.d.usb += out_fifo.r_en.eq(0)
            with m.FSM(reset="IDLE", domain="usb"):
                with m.State("IDLE"):
                    with m.If(pdm_transmitter.pcm_strobe_in & out_fifo.r_rdy):
                        m.d.usb += [
                            se16[0:8].eq(sample),
                            out_fifo.r_en.eq(1)
                        ]
                        m.next="B0ACK"
                with m.State("B0ACK"):
                    m.next="B1"
                with m.State("B1"):
                    with m.If(out_fifo.r_rdy):
                        with m.If(fbit):
                            # 1st byte again
                            m.d.usb += [
                                se16[0:8].eq(sample),
                                out_fifo.r_en.eq(1)
                            ]
                            m.next="B0ACK"
                        with m.Else():
                            m.d.usb += [
                                se16[8:16].eq(sample),
                                out_fifo.r_en.eq(1)
                            ]
                            m.next="B1ACK"
                with m.State("B1ACK"):
                    m.next="IDLE"

            m.d.comb += [
                # wire USB to PDM transmitter. Assume 28-bit PCM2PDM
                pdm_transmitter.pcm_data_in.eq(se16 << (28-16-3)),
            ]

        return m


elaboratable = LunaUSBAudioDevice()
name = 'LunaUSBAudioDevice'

ports = []

# Patch through all Records/Ports
for port_name, port in vars(elaboratable).items():
    if not port_name.startswith("_") and isinstance(port, (Signal, Record)):
        ports += port._lhs_signals()

verilog_text = verilog.convert(elaboratable, name=name, ports=ports, strip_internal_attrs=True)
verilog_file = f"verilog/{name}.v"

vdir = os.path.join(os.getcwd(), "verilog")
os.makedirs(vdir, exist_ok=True)

with open(verilog_file, "w") as f:
    f.write(verilog_text)
