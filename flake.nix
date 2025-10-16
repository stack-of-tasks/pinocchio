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
            eigen_5 = pkgs.eigen.overrideAttrs (super: rec {
              version = "5.0.0";
              src = pkgs.fetchFromGitLab {
                inherit (super.src) owner repo;
                tag = version;
                hash = "sha256-L1KUFZsaibC/FD6abTXrT3pvaFhbYnw+GaWsxM2gaxM=";
              };
              patches = [ ];
              postPatch = "";
            });
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

            libpinocchio-eigen_5 =
              (self'.packages.libpinocchio.override { eigen = self'.packages.eigen_5; }).overrideAttrs
                (super: {
                  pname = "${super.pname}-eigen_5";
                  cmakeFlags = super.cmakeFlags ++ [
                    "-DBUILD_WITH_CASADI_SUPPORT=OFF"
                    "-DBUILD_WITH_COLLISION_SUPPORT=OFF"
                  ];
                });
            pinocchio-py = pkgs.python3Packages.pinocchio.overrideAttrs (super: {
              pname = "pinocchio-py";
              cmakeFlags = super.cmakeFlags ++ [ "-DBUILD_STANDALONE_PYTHON_INTERFACE=ON" ];
              propagatedBuildInputs = super.propagatedBuildInputs ++ [
                pkgs.example-robot-data
                self'.packages.libpinocchio
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
