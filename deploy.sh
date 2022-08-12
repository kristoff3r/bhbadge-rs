#!/usr/bin/env bash

set -eu -o pipefail

cargo build --release

udisksctl mount -b /dev/disk/by-label/RPI-RP2

cargo run --release

while ! [ -e /dev/ttyACM0 ]; do sleep 0.05; done

sudo echo Connecting to console

# sudo stdbuf -o0 cat /dev/ttyACM0 | ~/.cargo/bin/defmt-print -e ./target/thumbv6m-none-eabi/debug/app
