# From source

This solution is the best choice if you want to work with the development version of Pinocchio or to contribute to the project with your fixes and improvements.

## Dependencies

Pinocchio has only few required dependencies and some optional ones which add more features, making Pinocchio a versatile framework.

### Required dependencies:

- Eigen3 for linear algebra. Version >= 3.0.5
- Boost with components filesystem, unit_test_framework and system. Version >= 1.34

### Optional dependencies:

- Boost with component python. Version >= 1.34. Required for the python bindings.
- EigenPy An efficient binding between Numpy and Eigen using boost::python. Version >= 1.3.0. Required for the python bindings.
- [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl) the Fast Collision Library. Version >= 0.4.1. Useful for collision detections.
- assimp for the reading of raw mesh files. Version >= 3.0.0.
- urdfdom for the reading of URDF models. Version >= 0.2.10.

## Installation

1. Clone the git repository:

```
git clone --recursive https://github.com/stack-of-tasks/pinocchio
```

2. Switch between the master or the devel branch:

```
git checkout master
```

or

```
git checkout devel
```

3. Move to pinocchio, create the build directory and move there:

```
cd pinocchio && mkdir build && cd build
```

4. Source with cmake with Release mode and set your desired installation directory:

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
```

if you want to use another python version, please add `-DPYTHON_EXECUTABLE=/usr/bin/python3`

5. Build the application:

```
make -j4
```

6. Install pinocchio:

```
make install
```

### Configure environment variables

All the packages will be installed in the `CMAKE_INSTALL_PREFIX` path, setting by default to point to `/usr/local`. To make use of installed libraries and programs, you must need to configure your `PATH`, `PKG_CONFIG_PATH`, `PYTHONPATH` and other similar environment variables to point inside this directory. For instance:

```
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH =/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```

Please check the path you're adding in PYTHONPATH. Depending on your system, it might use `pythonX` or
`pythonX.Y`, and `site-packages` or `dist-packages`.

You may directly add those lines to your `$HOME/.bashrc` for a persistent configuration.
