#!/usr/bin/env bash

set -eu -o pipefail

sleep 2
udisksctl mount -b /dev/disk/by-label/RPI-RP2
cargo run
