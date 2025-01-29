# Pinocchio examples in Python

This directory contains minimal examples on how to use **Pinocchio** with the Python bindings or directly in C++.
Some examples require the installation of external packages like 3D viewers. You can install them easily via **pip** or **conda**.

Installing **meshcat**:
```bash
pip install meshcat
```

Installing **gepetto-viewer**:
```bash
conda install gepetto-viewer-corba -c conda-forge
```

Installing **panda3d_viewer**:
```bash
pip install panda3d_viewer
```

Installing **RViz**:
```bash
conda install ros-noetic-rviz -c RoboStack
```

## Loading a model

- Loading an embeded Model: `python -i overview-simple.py` and in C++ `g++ overview-simple.cpp -o overview-simple $(pkg-config --cflags --libs pinocchio) && ./overview-simple`
- Loading a URDF model: `python -i overview-urdf.py` and in C++ `g++ -g overview-urdf.cpp -o overview-urdf $(pkg-config --cflags --libs pinocchio) && ./overview-urdf`
- Using RobotWrapper to encapsulate a URDF model: `python -i robot-wrapper-viewer.py`

## Simulating a model

-  Simulating a multiple pendulum: `python -i simulation-pendulum.py --with-cart -N=2`

## Computes analytical derivatives of rigid body dynamics algorithms

- Computing forward kinematics derivatives: `python -i kinematics-derivatives.py` and in C++ `g++ kinematics-derivatives.cpp -o kinematics-derivatives $(pkg-config --cflags --libs pinocchio) && ./kinematics-derivatives`
- Computing forward dynamics derivatives: `python -i forward-dynamics-derivatives.py` and in C++ `g++ -I -g forward-dynamics-derivatives.cpp -o forward-dynamics-derivatives $(pkg-config --cflags --libs pinocchio) && ./forward-dynamics-derivatives`
- Computing inverse dynamics derivatives: `python -i inverse-dynamics-derivatives.py` and in C++ `g++ inverse-dynamics-derivatives.cpp -o inverse-dynamics-derivatives $(pkg-config --cflags --libs pinocchio) && ./inverse-derivatives`

## Displaying the models

For the following examples, you should have [gepetto-gui](https://github.com/Gepetto/gepetto-viewer-corba) or [MeshCat](https://github.com/rdeits/meshcat) installed.

- Loading a robot model using [gepetto-gui](https://github.com/Gepetto/gepetto-viewer-corba): `python -i gepetto-viewer.py`
- Loading a robot model using [MeshCat](https://github.com/rdeits/meshcat): `python -i meshcat-viewer.py`
- Loading a model with basic geometries [MeshCat](https://github.com/rdeits/meshcat): `python -i sample-model-viewer.py`

## Collision checking

**Pinocchio** encapsulates  [FCL](https://github.com/coal-library/coal) in it. You can then do collision checking or distance computations with only few lines of code.

- Check collisions using [FCL](https://github.com/coal-library/coal): `python -i collisions.py` and in C++ `g++ collision.cpp -o collision $(pkg-config --cflags --libs pinocchio) && ./collision`

## Multiprecision arithmetic

Thanks to the full templatization of the project, **Pinocchio** is able to perform full precision arithmetic (via Boost.Multiprecision for instance).

- Multiprecision example: `g++ multiprecision.cpp -o multiprecision $(pkg-config --cflags --libs pinocchio) && ./multiprecision`

## Adding new examples

If you need a specific example for your applications, to not hesitate to open an issue detailing your specific request.
