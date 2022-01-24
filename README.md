# Luna UAC2 PDM

Status: Trial / Works on ButterStick

A LiteX module implementing a USB UAC2 module with simple PDM in/out. Based on @hansfbaier's [DECA USB Audio Interface](https://github.com/amaranth-community-unofficial/deca-usb2-audio-interface).
Making use of the LUNA USB stack, compiled into verilog to include directly into LiteX SoCs.


[> Caveat
---------

Only support monoral 48kHz sampling rate signals. Also assume 60MHz usb clock.

[> Links
--------

*[LUNA: USB Multitool and Gateware Library](https://luna.readthedocs.io/en/latest/index.html)

*[ButterStick](https://github.com/butterstick-fpga/butterstick-hardware)

*[USB2 High Speed Core](https://github.com/amaranth-community-unofficial/usb2-highspeed-core)

*[DECA USB Audio Interface](https://github.com/amaranth-community-unofficial/deca-usb2-audio-interface)

*[amlib](https://github.com/amaranth-community-unofficial/amlib)

*[PCM2PDM](https://github.com/kazkojima/pcm2pdm-example)

*[PDM2PCM](https://github.com/kazkojima/pdmmic-example)