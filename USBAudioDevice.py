# This file is Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
# License: BSD

import os

import litex
from migen import *

# Create a migen module to interface into a compiled nmigen module
class USBAudioDevice(Module):
    def __init__(self, platform, ulpi_pads, pdmout_pads, pdmin_pads):
        ulpi_data = TSTriple(8)

        reset = Signal()
        self.comb += ulpi_pads.rst.eq(~reset)
        
        vdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "verilog")
        platform.add_source(os.path.join(vdir, f"LunaUSBAudioDevice.v"))


        self.usb_tx = usb_tx = litex.soc.interconnect.stream.Endpoint([("data", 8)])
        self.usb_rx = usb_rx = litex.soc.interconnect.stream.Endpoint([("data", 8)])

        self.specials += ulpi_data.get_tristate(ulpi_pads.data)

        self.params = dict(
            # Clock / Reset
            i_clk_usb   = ClockSignal("usb"),
            i_clk_sync   = ClockSignal("sys"),
            i_rst_sync   = ResetSignal(),

            o_ulpi__data__o = ulpi_data.o,
            o_ulpi__data__oe = ulpi_data.oe,
            i_ulpi__data__i = ulpi_data.i,
            o_ulpi__clk__o = ulpi_pads.clk,
            o_ulpi__stp = ulpi_pads.stp,
            i_ulpi__nxt__i = ulpi_pads.nxt,
            i_ulpi__dir__i = ulpi_pads.dir,
            o_ulpi__rst = reset,

            o_pdmout__data__o = pdmout_pads.data,
            o_pdmout__clk__o = pdmout_pads.clk,
            i_pdmin__data__i = pdmin_pads.data,
            o_pdmin__clk__o = pdmin_pads.clk,
        )

        self.specials += Instance("LunaUSBAudioDevice",
            **self.params
        )

        
