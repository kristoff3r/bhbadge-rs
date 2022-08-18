{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, rust-overlay, ... }:
    let
      system = "x86_64-linux";
      overlays = [ (import rust-overlay) ];
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      pythonWithSerial = pkgs.python3.withPackages (ps: with ps; [pyserial]);
      connect-to-console = pkgs.writeScriptBin "connect-to-console" ''
        #!/usr/bin/env bash

        sudo echo Connecting to console
        sudo ${pythonWithSerial}/bin/python3 ${./serial.py} | ~/.cargo/bin/defmt-print -e ./target/thumbv6m-none-eabi/debug/app
      '';

    in
    {
      devShell."${system}" = with pkgs;
        mkShell {
          nativeBuildInputs = [
            # nix develop shells will by default include a bash in the $PATH,
            # however this bash will be a non-interactive bash. The deviates from
            # how nix-shell works. This fix was taken from:
            #    https://discourse.nixos.org/t/interactive-bash-with-nix-develop-flake/15486
            bashInteractive

            # rust. compile to wasm and linux
            (rust-bin.stable.latest.default.override {
              extensions = [ "rust-src" ];
              targets = [ "thumbv6m-none-eabi" ];
            })

            # Tools for flashing
            elf2uf2-rs flip-link python3

            # Tools for accessing the serial port (using a logic analyzer)
            sigrok-cli connect-to-console
          ];

          shellHook = ''
            # nix develop shells will by default overwrite the $SHELL variable with a
            # non-interactive version of bash. The deviates from how nix-shell works.
            # This fix was taken from:
            #    https://discourse.nixos.org/t/interactive-bash-with-nix-develop-flake/15486
            #
            # See also: nixpkgs#5131 nixpkgs#6091
            export SHELL=${bashInteractive}/bin/bash
          '';
        };
    };
}
