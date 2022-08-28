#!/usr/bin/env bash

set -eu -o pipefail

DEFMT_LOG=debug cargo build

udisksctl mount -b /dev/disk/by-label/RPI-RP2

DEFMT_LOG=debug cargo run --example $1

connect-to-console
