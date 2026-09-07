# Build and develop with pixi

The easiest way to set up a development environment is to use [pixi](https://pixi.sh/latest/#installation).

[pixi](https://pixi.sh/latest/) is a cross-platform package manager for developers.
It installs all required dependencies in the `.pixi` directory.
It's used by our CI, so you get the same stable and tested dependencies.

Run the following command to install dependencies, configure, build and test the project:

```bash
pixi run test
```

The project is built in the `build` directory.

The typical workflow is:

```bash
pixi shell
pixi run configure
ninja -C build
```

After `pixi run configure`, use `cmake` and `ninja` manually to reconfigure and build the project.

## Environments

The pixi manifest contains many environments. The most common ones are:

- **default**: core pinocchio, URDF parser and Python bindings
- **collision**: **default** + collision support (with coal)
- **all-benchmark**: all pinocchio features

To activate a specific environment, run:

```bash
pixi shell -e all-benchmark
```

Using **all-benchmark** makes it easy to choose which features to build.
In this case, use the following CMake options:

- `BUILD_WITH_URDF_SUPPORT`: URDF parser support
- `BUILD_WITH_SDF_SUPPORT`: SDF parser support
- `BUILD_WITH_COLLISION_SUPPORT`: collision support
- `BUILD_WITH_AUTODIFF_SUPPORT`: automatic differentiation support with CppAD
- `BUILD_WITH_CASADI_SUPPORT`: CasADi support
- `BUILD_WITH_CODEGEN_SUPPORT`: code generation support with CppADCodeGen (no Windows support)
- `BUILD_WITH_OPENMP_SUPPORT`: parallel algorithms with OpenMP
- `BUILD_WITH_EXTRA_SUPPORT`: extra features support
- `BUILD_WITH_ACCELERATE_SUPPORT`: Apple Accelerate backend support (macOS only)
- `BUILD_PYTHON_INTERFACE`: Python bindings
- `BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT`: MPFR support in the Python bindings
- `BUILD_PYTHON_BINDINGS_WITH_FLOAT32_SUPPORT`: float32 support in the Python bindings
- `GENERATE_PYTHON_STUBS`: Python stubs generation
- `BUILD_BENCHMARK`: benchmarks

With the **all-benchmark** environment, all these options are ON.
To turn one off, pass the corresponding `-D` flag to `cmake`:

```bash
cmake -B build -DBUILD_WITH_SDF_SUPPORT=OFF
```

## Faster build

When you work on a single feature with one associated test, turn off
the explicit template instantiation. This avoids building the whole library.

- Set `ENABLE_TEMPLATE_INSTANTIATION` to OFF: `cmake -B build -DENABLE_TEMPLATE_INSTANTIATION=OFF`
- Build and run the corresponding test:
```bash
ninja -C build pinocchio-test-cpp-<name>
ctest --test-dir build --output-on-failure -R pinocchio-test-cpp-<name>
```
