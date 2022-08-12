import sys
import os
import os.path
import time
import serial

while not os.path.exists("/dev/ttyACM0"):
    time.sleep(0.001)

with serial.Serial('/dev/ttyACM0') as ser:
    while True:
        byte = ser.read(1)
        sys.stdout.buffer.write(byte)
        sys.stdout.flush()
