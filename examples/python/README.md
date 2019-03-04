# Pinocchio examples in Python

This directory contains minimal examples on how to use **Pinocchio** with the Python bindings. 

## Loading a model

- Loading a URDF model: `python -i load-urdf.py`

## Computing analytical derivatives of rigid body dynamics algorithms

- Computing forward kinematics derivatives: `python -i kinematics-derivatives.py` 
- Computing forward dynamics (fd) derivatives: `python -i fd-derivatives.py` 
- Computing inverse dynamics (id) derivatives: `python -i id-derivatives.py` 

## Viewer with Pinocchio

- Loading a robot model using [gepetto-gui](https://github.com/Gepetto/gepetto-viewer-corba): `python -i gepetto-viewer.py`
- Loading a robot model using [MeshCat](https://github.com/rdeits/meshcat): `python -i meshcat-viewer.py`

## Collisions

- Check collisions using [FCL](https://github.com/humanoid-path-planner/hpp-fcl): `python -i collision-detection.py`
