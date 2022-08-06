#!/bin/sh

nix-shell --run "sleep 2 && udisksctl mount -b /dev/disk/by-label/RPI-RP2 && cargo run"
