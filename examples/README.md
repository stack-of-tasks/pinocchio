# Pinocchio examples in Python

This directory contains minimal examples on how to use **Pinocchio** with the Python bindings or directly in C++. 

## Loading a model

- Loading an embeded Model: `python -i overview-simple.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g overview-simple.cpp -o overview-simple && ./overview-simple`
- Loading a URDF model: `python -i overview-urdf.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g overview-urdf.cpp -o overview-urdf && ./overview-urdf`
- Using RobotWrapper to encapsulate a URDF model: `python -i robot-wrapper-viewer.py`

## Computes analytical derivatives of rigid body dynamics algorithms

- Computing forward kinematics derivatives: `python -i kinematics-derivatives.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g kinematics-derivatives.cpp -o kinematics-derivatives && ./kinematics-derivatives`
- Computing forward dynamics derivatives: `python -i forward-dynamics-derivatives.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g forward-dynamics-derivatives.cpp -o forward-dynamics-derivatives && ./forward-dynamics-derivatives`
- Computing inverse dynamics derivatives: `python -i inverse-dynamics-derivatives.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g inverse-dynamics-derivatives.cpp -o inverse-dynamics-derivatives && ./inverse-derivatives`

## Displaying the models

For the following examples, you should have [gepetto-gui](https://github.com/Gepetto/gepetto-viewer-corba) or [MeshCat](https://github.com/rdeits/meshcat) installed.

- Loading a robot model using [gepetto-gui](https://github.com/Gepetto/gepetto-viewer-corba): `python -i gepetto-viewer.py`
- Loading a robot model using [MeshCat](https://github.com/rdeits/meshcat): `python -i meshcat-viewer.py`
- Loading a model with basic geometries [MeshCat](https://github.com/rdeits/meshcat): `python -i sample-model-viewer.py`

## Collision checking

**Pinocchio** encapsulates  [FCL](https://github.com/humanoid-path-planner/hpp-fcl) in it. You can then do collision checking or distance computations with only few lines of code.

- Check collisions using [FCL](https://github.com/humanoid-path-planner/hpp-fcl): `python -i collisions.py` and in C++ `g++ -I $(pkg-config --cflags pinocchio) -g collision.cpp -o collision && ./collision`

## Multiprecision arithmetic

Thanks to the full templatization of the project, **Pinocchio** is able to perform full precision arithmetic (via Boost.Multiprecision for instance).

- Multiprecision example: `g++ -I $(pkg-config --cflags pinocchio) -g multiprecision.cpp -o multiprecision && ./multiprecision`

## Adding new examples

If you need a specific example for your applications, to not hesitate to open an issue detailing your specific request.
