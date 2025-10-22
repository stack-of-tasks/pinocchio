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
            program = pkgs.python3.withPackages (p: [
              self'.packages.default
              p.example-robot-data
              p.meshcat
              p.viser
            ]);
          };
          packages = {
            default = self'.packages.pinocchio;
            pinocchio = pkgs.python3Packages.pinocchio.overrideAttrs (super: {
              propagatedBuildInputs = super.propagatedBuildInputs ++ [ pkgs.example-robot-data ];
              nativeCheckInputs = [ pkgs.python3Packages.pybind11 ];
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
            libpinocchio = pkgs.pinocchio.overrideAttrs (super: {
              pname = "libpinocchio";
              propagatedBuildInputs = super.propagatedBuildInputs ++ [ pkgs.example-robot-data ];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./benchmark
                  # ./bindings
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
            pinocchio-py = pkgs.python3Packages.pinocchio.overrideAttrs (super: {
              pname = "pinocchio-py";
              cmakeFlags = super.cmakeFlags ++ [ "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON" ];
              propagatedBuildInputs = super.propagatedBuildInputs ++ [
                pkgs.example-robot-data
                self'.packages.libpinocchio
              ];
              nativeCheckInputs = [ pkgs.python3Packages.pybind11 ];
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
                  # ./src
                  ./unittest
                  ./utils
                ];
              };
            });
          };
        };
    };
}
