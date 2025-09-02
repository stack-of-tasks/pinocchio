{
  description = "Fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives.";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        {
          inputs',
          pkgs,
          self',
          system,
          ...
        }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          packages = {
            default = self'.packages.pinocchio;
            pinocchio = pkgs.python3Packages.pinocchio.overrideAttrs (super: {
              propagatedBuildInputs = super.propagatedBuildInputs ++ [ pkgs.example-robot-data ];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./benchmark
                  ./bindings
                  ./CMakeLists.txt
                  ./doc
                  ./examples
                  ./include
                  ./models
                  ./package.xml
                  ./sources.cmake
                  ./src
                  ./unittest
                  ./utils
                ];
              };
            });
          };
        };
    };
}
