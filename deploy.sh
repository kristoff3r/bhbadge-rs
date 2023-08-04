#!/usr/bin/env bash

set -eu -o pipefail


if [ -z ${NOLOG+x} ]; then
  export DEFMT_LOG=debug
fi

if [ -z ${RELEASE+x} ]; then
  cargo build --example $1
else
  cargo build --example $1 --release
fi

sudo echo Connecting to console

udisksctl unmount -b /dev/disk/by-label/RPI-RP2 2>/dev/null || true
udisksctl mount -b /dev/disk/by-label/RPI-RP2

if [ -z ${RELEASE+x} ]; then
  cargo run --example $1
  connect-to-console debug/examples/$1
else
  cargo run --example $1 --release
  connect-to-console release/examples/$1
fi

