#!/usr/bin/env python3
#
# Copyright (c) 2022 Kaz Kojima <kkojima@rr.iij4u.or.jp>
# SPDX-License-Identifier: BSD--3-Clause

import os

from amaranth import *
from amaranth.lib.cdc    import FFSynchronizer

from amaranth.hdl.rec import DIR_FANIN, DIR_FANOUT, DIR_NONE
from amaranth.back import verilog

from amaranth.lib.fifo  import SyncFIFO, AsyncFIFO
from amlib.stream       import StreamInterface, connect_stream_to_fifo, connect_fifo_to_stream
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
#from luna.full_devices import USBAudioDevice as LunaDeviceACM
from luna.gateware.architecture.car import PHYResetController

from pdm.pcm2pdm import PCM2PDM

from requesthandlers import UAC2RequestHandlers

class ChannelsToUSBStream(Elaboratable):
    def __init__(self, max_nr_channels=1, sample_width=16, max_packet_size=512):
        assert sample_width in [16, 24, 32]

        # parameters
        self._max_nr_channels = max_nr_channels
        self._channel_bits    = Shape.cast(range(max_nr_channels)).width
        self._sample_width    = sample_width
        self._max_packet_size = max_packet_size

        # ports
        self.usb_stream_out      = StreamInterface()
        self.channel_stream_in   = StreamInterface(name="channels_stream_in", payload_width=self._sample_width, extra_fields=[("channel_no", self._channel_bits)])

    def elaborate(self, platform):
        m = Module()
        m.submodules.out_fifo = out_fifo = SyncFIFO(width=8, depth=self._max_packet_size)

        channel_stream  = self.channel_stream_in
        channel_payload = Signal(self._sample_width)
        channel_valid   = Signal()
        channel_ready   = Signal()

        m.d.comb += [
            *connect_fifo_to_stream(out_fifo, self.usb_stream_out),
            channel_payload.eq(channel_stream.payload),
            channel_valid.eq(channel_stream.valid),
            channel_stream.ready.eq(channel_ready),
        ]

        current_sample  = Signal(32 if self._sample_width > 16 else 16)
        current_channel = Signal(self._channel_bits)
        current_byte    = Signal(2 if self._sample_width > 16 else 1)

        last_channel    = self._max_nr_channels - 1
        num_bytes = 4
        last_byte = num_bytes - 1

        shift = 8 if self._sample_width == 24 else 0

        with m.If(out_fifo.w_rdy):
            with m.FSM() as fsm:
                current_channel_next = (current_channel + 1)[:self._channel_bits]

                with m.State("WAIT-FIRST"):
                    # we have to accept data until we find a first channel sample
                    m.d.comb += channel_ready.eq(1)
                    with m.If(channel_valid & (channel_stream.channel_no == 0)):
                        m.d.sync += [
                            current_sample.eq(channel_payload << shift),
                            current_channel.eq(0),
                        ]
                        m.next = "SEND"

                with m.State("SEND"):
                    m.d.comb += [
                        out_fifo.w_data.eq(current_sample[0:8]),
                        out_fifo.w_en.eq(1),
                    ]
                    m.d.sync += [
                        current_byte.eq(current_byte + 1),
                        current_sample.eq(current_sample >> 8),
                    ]

                    with m.If(current_byte == last_byte):
                        with m.If(channel_valid):
                            m.d.comb += channel_ready.eq(1)

                            m.d.sync += current_channel.eq(current_channel_next)

                            with m.If(current_channel_next == channel_stream.channel_no):
                                m.d.sync += current_sample.eq(channel_payload << shift)
                                m.next = "SEND"
                            with m.Else():
                                m.next = "FILL-ZEROS"

                        with m.Else():
                            m.next = "WAIT"

                with m.State("WAIT"):
                    with m.If(channel_valid):
                        m.d.comb += channel_ready.eq(1)
                        m.d.sync += [
                            current_sample.eq(channel_payload << shift),
                            current_channel.eq(current_channel_next),
                        ]
                        m.next = "SEND"

                with m.State("FILL-ZEROS"):
                    m.d.comb += [
                        out_fifo.w_data.eq(0),
                        out_fifo.w_en.eq(1),
                    ]
                    m.d.sync += current_byte.eq(current_byte + 1)

                    with m.If(current_byte == last_byte):
                        m.d.sync += current_channel.eq(current_channel + 1)
                        with m.If(current_channel == last_channel):
                            m.next = "WAIT-FIRST"
        return m

class USBStreamToChannels(Elaboratable):
    def __init__(self, max_nr_channels=1):
        # parameters
        self._max_nr_channels = max_nr_channels
        self._channel_bits    = 0#Shape.cast(range(max_nr_channels)).width

        # ports
        self.usb_stream_in       = StreamInterface(name="usb_stream")
        self.channel_stream_out  = StreamInterface(name="channel_stream", payload_width=16, extra_fields=[("channel_no", self._channel_bits)])

    def elaborate(self, platform):
        m = Module()

        out_channel_no   = Signal(self._channel_bits)
        out_sample       = Signal(8)
        usb_valid        = Signal()
        usb_first        = Signal()
        usb_payload      = Signal(8)
        out_ready        = Signal()

        m.d.comb += [
            usb_first.eq(self.usb_stream_in.first),
            usb_valid.eq(self.usb_stream_in.valid),
            usb_payload.eq(self.usb_stream_in.payload),
            out_ready.eq(self.channel_stream_out.ready),
            self.usb_stream_in.ready.eq(out_ready),
        ]

        m.d.sync += [
            self.channel_stream_out.valid.eq(0),
            self.channel_stream_out.first.eq(0),
            self.channel_stream_out.last.eq(0),
        ]

        with m.If(usb_valid & out_ready):
            with m.FSM():
                with m.State("B0"):
                    #with m.If(usb_first):
                    #    m.d.sync += out_channel_no.eq(0)
                    #with m.Else():
                    #    m.d.sync += out_channel_no.eq(out_channel_no + 1)
                    m.d.sync += out_channel_no.eq(0)

                    m.next = "B1"

                with m.State("B1"):
                    m.d.sync += out_sample.eq(usb_payload)
                    m.next = "B2"

                with m.State("B2"):
                    m.d.sync += [
                        self.channel_stream_out.payload.eq(Cat(out_sample, usb_payload)),
                        self.channel_stream_out.valid.eq(1),
                        self.channel_stream_out.channel_no.eq(out_channel_no),
                        self.channel_stream_out.first.eq(out_channel_no == 0),
                        self.channel_stream_out.last.eq(out_channel_no == (2**self._channel_bits - 1)),
                    ]
                    m.next = "B0"

        return m


class LunaUSBAudioDevice(Elaboratable):
    """ USB Audio Class v2 interface """
    NR_CHANNELS = 1
    MAX_PACKET_SIZE = 512 # NR_CHANNELS * 24 + 4
    USE_ILA = False
    ILA_MAX_PACKET_SIZE = 512

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
        typeIStreamingInterface.bSubslotSize   = 4
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

        # Windows wants a stereo pair as default setting, so let's have it
        self.create_input_streaming_interface(c, nr_channels=self.NR_CHANNELS, alt_setting_nr=1, channel_config=0x3)


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

        self.pdmout = Record(
            [
                ('data', [('o', 1, DIR_FANOUT)]),
                ('clk', [('o', 1, DIR_FANOUT)]),
                ('cs', [('o', 1, DIR_FANOUT)]),
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
        
        m.submodules.pdm_transmitter = pdm_transmitter = PCM2PDM()

        sout = Signal()
        cout = Signal()
        csout = Signal(reset=1)
        m.d.comb += [
            # wire up PDM transmitter
            self.pdmout.data.eq(pdm_transmitter.pdm_data_out),
            self.pdmout.clk.eq(pdm_transmitter.pdm_clock_out),
            #self.pdmout.data.eq(sout),
            #self.pdmout.clk.eq(cout),
            #self.pdmout.cs.eq(csout),
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
        audio_in_frame_bytes = Signal(range(self.MAX_PACKET_SIZE), reset=16 * self.NR_CHANNELS)
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
            ep2_in.bytes_in_frame.eq(audio_in_frame_bytes),
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

        m.submodules.usb_to_channel_stream = usb_to_channel_stream = \
            DomainRenamer("usb")(USBStreamToChannels(self.NR_CHANNELS))

        m.submodules.channels_to_usb_stream = channels_to_usb_stream = \
            DomainRenamer("usb")(ChannelsToUSBStream(self.NR_CHANNELS))

        m.submodules.out_fifo = out_fifo = \
            AsyncFIFO(width=8, depth=512, r_domain="sync", w_domain="usb")

        m.d.comb += connect_stream_to_fifo(ep1_out.stream, out_fifo)
        se16 = Signal(signed(16))
        sample = Signal(8)
        m.d.comb += sample.eq(out_fifo.r_data)
        m.d.sync += out_fifo.r_en.eq(0)
        with m.FSM(reset="IDLE"):
            with m.State("IDLE"):
                with m.If(pdm_transmitter.pcm_strobe_in & out_fifo.r_rdy):
                    m.d.sync += [
                        se16[0:8].eq(sample),
                        out_fifo.r_en.eq(1)
                    ]
                    m.next="B0ACK" 
            with m.State("B0ACK"):
                m.next="B1"
            with m.State("B1"):
                with m.If(out_fifo.r_rdy):
                    m.d.sync += [
                        se16[8:16].eq(sample),
                        out_fifo.r_en.eq(1)
                    ]
                    m.next="B1ACK"
            with m.State("B1ACK"):
                m.next="IDLE"

        m.d.comb += [
            # wire USB to PDM transmitter via fifo
            #usb_to_channel_stream.usb_stream_in.stream_eq(ep1_out.stream),
            #connect_stream_to_fifo(usb_to_channel_stream.channel_stream_out,
            #                       out_fifo),
            #out_fifo.r_en.eq(pdm_transmitter.pcm_strobe_in),
            pdm_transmitter.pcm_data_in.eq(se16 << (28-16-3)),
        ]

        spi = Signal(16)
        scount = Signal(range(16))
        bcount = Signal(range(16))
        m.d.comb += sout.eq(spi[16-1])
        with m.FSM(reset="SIDLE"):
            with m.State("SIDLE"):
                with m.If(pdm_transmitter.pcm_strobe_in):
                    m.d.sync += [
                        #pdm_transmitter.pcm_data_in.eq(se16 << (28-16-3)),
                        #out_fifo.r_en.eq(1),
                        spi.eq(se16),
                        scount.eq(0),
                        bcount.eq(0),
                        cout.eq(0),
                        csout.eq(0)
                    ]
                    m.next="SO"
            with m.State("SO"):
                with m.If(scount == 7):
                    m.d.sync += cout.eq(1)
                with m.If(scount == 15):
                    m.d.sync += [
                        cout.eq(0),
                        spi.eq(spi << 1),
                        bcount.eq(bcount+1)
                    ]
                    with m.If(bcount == 15):
                        m.d.sync += csout.eq(1)
                        m.next="SIDLE"
                m.d.sync += scount.eq(scount+1)

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
