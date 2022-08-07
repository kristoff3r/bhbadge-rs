import sys

for line in sys.stdin:
    if line.startswith('uart-1'):
        sys.stdout.buffer.write(bytes.fromhex(line.split()[1]))
        sys.stdout.flush()
