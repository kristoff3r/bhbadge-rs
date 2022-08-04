#!/bin/sh

nix-shell -p elf2uf2-rs flip-link --run "udisksctl mount -b /dev/disk/by-label/RPI-RP2 && cargo run"
