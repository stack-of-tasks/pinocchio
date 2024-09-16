{
  description = "Fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives.";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    #nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    # use gepetto fork until https://github.com/NixOS/nixpkgs/pull/337942
    nixpkgs.url = "github:gepetto/nixpkgs";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.pinocchio;
            pinocchio = pkgs.python3Packages.pinocchio.overrideAttrs (super: {
              # avoid SIGTRAP on macos github runners
              cmakeFlags = super.cmakeFlags ++ pkgs.lib.optionals pkgs.stdenv.isDarwin [
                "-DCMAKE_CTEST_ARGUMENTS=--exclude-regex;pinocchio-example-py-casadi-quadrotor-ocp"
              ];
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
