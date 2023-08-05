{ rustPlatform, fetchCrate, pkg-config }:

rustPlatform.buildRustPackage rec {
  pname = "defmt-print";
  version = "0.3.9";

  src = fetchCrate {
    inherit pname version;
    sha256 = "sha256-4uNvCDeLpQoLpgJEHn+/JIKXHVuZeTB1GebulA8hKn0=";
  };

  nativeBuildInputs = [
    pkg-config
  ];

  cargoSha256 = "sha256-U0F9v8V9LcfdDCkVSteIMveFsweiYt+eECdUqxUOxa0=";
}
