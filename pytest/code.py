import time
print("TEST LOL")
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
This test will initialize the display using displayio and draw a solid green
background, a smaller purple rectangle, and some yellow text.
"""

import os

import board
import terminalio
import displayio
import digitalio
import pwmio
import time
from adafruit_display_text import label

# Release any resources currently in use for the displays
displayio.release_displays()


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
    b"\x36\x01\xC0"  # _MADCTL Default rotation plus BGR encoding
)


# pylint: disable=too-few-public-methods
# class ST7735R(displayio.Display):
#     """
#     ST7735R display driver
#     :param displayio.FourWire bus: bus that the display is connected to
#     :param bool bgr: (Optional) An extra init sequence to append (default=False)
#     :param bool invert: (Optional) Invert the colors (default=False)
#     """
#
#     def __init__(
#         self,
#         bus: displayio.FourWire,
#         *,
#         bgr: bool = False,
#         invert: bool = False,
#         **kwargs: Any
#     ):
#
#         init_sequence = _INIT_SEQUENCE
#         if bgr:
#             init_sequence += (
#                 b"\x36\x01\xC0"  # _MADCTL Default rotation plus BGR encoding
#             )
#         else:
#             init_sequence += (
#                 b"\x36\x01\xC8"  # _MADCTL Default rotation plus RGB encoding
#             )
#         if invert:
#             init_sequence += b"\x21\x00"  # _INVON
#         super().__init__(bus, init_sequence, **kwargs)

spi = board.SPI()

bl = pwmio.PWMOut(board.PWM0, frequency=5000, duty_cycle=0)
bl.duty_cycle = 60000


# display_bus = displayio.FourWire(
# 	spi, command=board.D1, chip_select=board.CS, reset=board.D0
# )

# def send_command(cmd, data):
#     display_bus.send(cmd, data)

tft_cs = digitalio.DigitalInOut(board.CS)
tft_cs.direction = digitalio.Direction.OUTPUT
tft_cs.value = True
tft_dc = digitalio.DigitalInOut(board.D1)
tft_dc.direction = digitalio.Direction.OUTPUT
tft_dc.value = True
tft_reset = digitalio.DigitalInOut(board.D0)
tft_reset.direction = digitalio.Direction.OUTPUT
tft_reset.value = True

def send_command(cmd, data):
    # Transaction
    tft_cs.value = False

    # Toggle command
    tft_dc.value = False

    # Send command
    spi.write(bytearray([cmd]))
    tft_cs.value = True
    time.sleep(1 / 1_000_000.0)
    tft_cs.value = False

    # Toggle data
    tft_dc.value = True
    spi.write(data)

    # End transaction
    tft_cs.value = True

def init_display():
    while not spi.try_lock():
        pass

    spi.configure(baudrate=24_000_000, phase=0, polarity=0)
    # Hard reset
    tft_cs.value = False
    tft_reset.value = False
    time.sleep(50 / 1000.0)
    tft_reset.value = False
    time.sleep(50 / 1000.0)
    tft_reset.value = True
    time.sleep(150 / 1000.0)
    tft_cs.value = True


    i = 0
    while i < len(_INIT_SEQUENCE):
        cmd = _INIT_SEQUENCE[i]
        data_size = _INIT_SEQUENCE[i+1]
        delay = data_size & 0x80
        data_size = data_size & 0x7f
        i += 2

        send_command(cmd, _INIT_SEQUENCE[i:i+data_size])

        i += data_size

        delay_len_ms = 10
        if delay > 0:
            delay_len_ms = int(_INIT_SEQUENCE[i])
            i += 1
            if delay_len_ms == 255:
                delay_len_ms = 500

        time.sleep(delay_len_ms / 1000.0)

# display = ST7735R(display_bus, width=128, height=160, rotation=0, bgr=True, colstart=2, rowstart=1, auto_refresh = False)
init_display()

color = 0
while True:
    send_command(0x2a, b"\x00\x10\x00\x20")
    time.sleep(0.01)
    send_command(0x2b, b"\x00\x10\x00\x20")
    time.sleep(0.01)
    send_command(0x2c, bytearray([color, color] * 17*17))
    time.sleep(0.01)
    color = (color + 1) % 256

while True:
    pass