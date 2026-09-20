{
  description = "struo development shell with shared Cargo caching";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nix-packages = {
      url = "github:tignear/nix-packages";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      nixpkgs,
      rust-overlay,
      nix-packages,
      ...
    }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
      environments = forAllSystems (
        system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ rust-overlay.overlays.default ];
          };
          rust = (pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml).override {
            extensions = [
              "clippy"
              "rustfmt"
              "rust-src"
              "rust-analyzer"
            ];
          };
          mbx = nix-packages.packages.${system}.mbx;
          # mbx dispatches on argv[0]; real Cargo remains later on PATH.
          cargoShim = pkgs.runCommand "struo-cargo-shim" { } ''
            mkdir -p $out/bin
            ln -s ${mbx}/bin/mbx $out/bin/cargo
          '';
        in
        {
          inherit
            pkgs
            rust
            mbx
            cargoShim
            ;
        }
      );
    in
    {
      packages = forAllSystems (system: {
        inherit (environments.${system}) mbx;
      });
      devShells = forAllSystems (
        system:
        let
          e = environments.${system};
        in
        {
          default = e.pkgs.mkShell {
            packages = [
              e.cargoShim
              e.rust
              e.mbx
            ]
            ++ (with e.pkgs; [
              stdenv.cc
              clang
              cmake
              gnumake
              pkg-config
              openssl
              git
              curl
              jq
              python3
              fuse-overlayfs
              direnv
              nix-direnv
            ]);
          };
        }
      );
      formatter = forAllSystems (system: environments.${system}.pkgs.nixfmt);
    };
}
