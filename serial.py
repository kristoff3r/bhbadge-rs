import serial, sys

with serial.Serial('/dev/ttyACM0') as ser:
    while True:
        byte = ser.read(1)
        sys.stdout.buffer.write(byte)
        sys.stdout.flush()
