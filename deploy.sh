#!/usr/bin/env bash

set -eu -o pipefail

sleep 2
udisksctl mount -b /dev/disk/by-label/RPI-RP2

DEFMT_LOG=debug cargo build
sudo echo sudo successful
DEFMT_LOG=debug cargo run

sleep 1000000 | sudo sigrok-cli --config samplerate=1M -t D0=r --driver=fx2lafw --channels D0 --continuous -P uart:baudrate=115200:rx=D0 -A uart=rx_data | python decode.py | ~/.cargo/bin/defmt-print -e ./target/thumbv6m-none-eabi/debug/app
