_INIT_SEQUENCE = bytearray(
    b"\x01\x80\x96"  # SWRESET and Delay 150ms
    b"\x11\x80\xff"  # SLPOUT and Delay
    b"\xb1\x03\x01\x2C\x2D"  # _FRMCTR1
    b"\xb2\x03\x01\x2C\x2D"  # _FRMCTR2
    b"\xb3\x06\x01\x2C\x2D\x01\x2C\x2D"  # _FRMCTR3
    b"\xb4\x01\x07"  # _INVCTR line inversion
    b"\xc0\x03\xa2\x02\x84"  # _PWCTR1 GVDD = 4.7V, 1.0uA
    b"\xc1\x01\xc5"  # _PWCTR2 VGH=14.7V, VGL=-7.35V
    b"\xc2\x02\x0a\x00"  # _PWCTR3 Opamp current small, Boost frequency
    b"\xc3\x02\x8a\x2a"
    b"\xc4\x02\x8a\xee"
    b"\xc5\x01\x0e"  # _VMCTR1 VCOMH = 4V, VOML = -1.1V
    b"\x20\x00"  # _INVOFF
    b"\x36\x01\x18"  # _MADCTL bottom to top refresh
    # 1 clk cycle nonoverlap, 2 cycle gate rise, 3 sycle osc equalie,
    # fix on VTL
    b"\x3a\x01\x05"  # COLMOD - 16bit color
    b"\xe0\x10\x02\x1c\x07\x12\x37\x32\x29\x2d\x29\x25\x2B\x39\x00\x01\x03\x10"  # _GMCTRP1 Gamma
    b"\xe1\x10\x03\x1d\x07\x06\x2E\x2C\x29\x2D\x2E\x2E\x37\x3F\x00\x00\x02\x10"  # _GMCTRN1
    b"\x13\x80\x0a"  # _NORON
    b"\x29\x80\x64"  # _DISPON
)

i = 0
while i < len(_INIT_SEQUENCE):
    cmd = _INIT_SEQUENCE[i]
    data_size = _INIT_SEQUENCE[i+1]
    delay = data_size & 0x80
    data_size = data_size & 0x7f
    i += 2

    # Transaction
    # tft_cs.value = False

    # # Toggle command
    # tft_dc.value = False

    # Send command
    # spi.write(cmd)
    # tft_cs.value = True
    # utime.sleep_us(1)
    # tft_cs.value = False

    # Toggle data
    # tft_dc.value = True
    # for b in _INIT_SEQUENCE[i:i+data_size]:
    #     spi.write(b)

    # End transaction
    # tft_cs.value = True

    i += data_size

    delay_len_ms = 10
    if delay > 0:
        delay_len_ms = int(_INIT_SEQUENCE[i])
        i += 1
        if delay_len_ms == 255:
            delay_len_ms = 500
        print("delay", delay_len_ms)
    print(i)
