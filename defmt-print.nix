{ lib, stdenv, rustPlatform, fetchCrate, pkg-config, libudev }:

rustPlatform.buildRustPackage rec {
  pname = "defmt-print";
  version = "0.3.2";

  src = fetchCrate {
    inherit pname version;
    sha256 = "sha256-Ud1sDysj6csh69lwJTIqpdqbb01EHQB39gAi+Fts5EM=";
  };

  nativeBuildInputs = [
    pkg-config
  ];

  cargoSha256 = "sha256-LXbApG+hvCUCAcQchDPNWBcNSzfpNlHcFebq+q5JEE0=";
}
