#!/bin/sh
set -eu -o pipefail

sudo echo sudo successful

sleep 1000000 | sudo sigrok-cli --config samplerate=1M -t D0=r --driver=fx2lafw --channels D0 --continuous -P uart:baudrate=115200:rx=D0 -A uart=rx_data | python decode.py | ~/.cargo/bin/defmt-print -e ./target/thumbv6m-none-eabi/debug/app
