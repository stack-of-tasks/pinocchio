Pinocchio: a C++ library for efficient Rigid Multi-body Dynamics computations
===========

[![License LGPL 3](https://img.shields.io/badge/license-LGPLv3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0.txt)
[![Build Status](https://travis-ci.org/stack-of-tasks/pinocchio.svg?branch=devel)](https://travis-ci.org/stack-of-tasks/pinocchio)
[![Coverage Status](https://coveralls.io/repos/github/stack-of-tasks/pinocchio/badge.svg?branch=devel)](https://coveralls.io/github/stack-of-tasks/pinocchio?branch=devel)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/7824/badge.svg)](https://scan.coverity.com/projects/pinocchio)

**Pinocchio** instatiates state-of-the-art Rigid Body Algotithms for poly-articulated systems based on revisited Roy Featherstone's algorithms.
In addition, **Pinocchio** instantiates analytical derivatives of the main Rigid-Body Algorithms like the Recursive Newton-Euler Algorithms or the Articulated-Body Algorithm.
**Pinocchio** is first tailored for legged robotics applications, but it can be used in extra contextes.
It is built upon Eigen for linear algebra and FCL for collision detections. **Pinocchio** comes with a Python interface for fast code protyping.

**Pinocchio** is now at the hearth of various robotics softwares as the [Stack-of-Tasks](http://stack-of-tasks.github.io) or the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc).

## Installation

**Pinocchio** can be easily installed on various Linux and Unix distributions. Please refer to the [installation procedure](http://stack-of-tasks.github.io/pinocchio/download.html).

## Examples

We provide some basic examples on how to use **Pinocchio** in Python in the [examples/python](./examples/python/REAME.md) directory.

## Tutorials 

**Pinocchio** is comming with a large bunch of tutorials aiming at introducting the basic tools for robotics control.
The content of the tutorials are described [here](http://projects.laas.fr/gepetto/index.php/Teach/Supaero2018) and the source code of these tutorials is located [here](https://github.com/stack-of-tasks/pinocchio-tutorials).

## Dependencies

**Pinocchio** depends on few additional packages which have to be available on your machine.

### Build dependencies
   - cmake (version >= 2.6)
   - pkg-config
   - Boost with components unit_test_framework
   - G++/CLANG
   
### Core dependencies
   - Eigen3 (version >= 3.0.5)   
   - Boost with components filesystem 
   
### Optional dependencies
   - urdfdom (version >= 0.2)
   - LUA 5.1
   - [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl)
   
### Python bindings
   - Python 2.7 or 3.0
   - Numpy
   - [EigenPy](https://github.com/stack-of-tasks/eigenpy.git)
   - Boost Python

## Citing Pinocchio

To cite **Pinocchio** in your academic research, please use the following bibtex line:
```
@misc{pinocchioweb,
      author = {Justin Carpentier and Florian Valenza and Nicolas Mansard and others},
      title = {Pinocchio: fast forward and inverse dynamics for poly-articulated systems},
      howpublished = {https://stack-of-tasks.github.io/pinocchio},
      year = {2015--2018}
     }
```
The algorithms for the analytical derivatives of rigid-body dynamics algorithms are detailed here:
```
@inproceedings{carpentier2018analytical,
  title={Analytical Derivatives of Rigid Body Dynamics Algorithms},
  author={Carpentier, Justin and Mansard, Nicolas},
  booktitle={Robotics: Science and Systems},
  year={2018}
}
```

## Questions

You need to ask a question? You may either directly post a [new issue](https://github.com/stack-of-tasks/pinocchio/issues) or use the mailing list <pinocchio@laas.fr>

## Acknowledgments

The development of **Pinocchio** is supported by the [Gepetto team](http://projects.laas.fr/gepetto/) @LAAS-CNRS.
