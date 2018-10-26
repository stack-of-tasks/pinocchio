Pinocchio: a C++ library for efficient Rigid Multi-body Dynamics computations
===========

[![License LGPL 3](https://img.shields.io/badge/license-LGPLv3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0.txt)
[![Build Status](https://travis-ci.org/stack-of-tasks/pinocchio.svg?branch=devel)](https://travis-ci.org/stack-of-tasks/pinocchio)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/pinocchio/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/pinocchio/master/coverage/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/7824/badge.svg)](https://scan.coverity.com/projects/pinocchio)

**Pinocchio** instantiates state-of-the-art Rigid Body Algorithms for poly-articulated systems based on revisited Roy Featherstone's algorithms.
In addition, **Pinocchio** instantiates analytical derivatives of the main Rigid-Body Algorithms like the Recursive Newton-Euler Algorithms or the Articulated-Body Algorithm.
**Pinocchio** is first tailored for legged robotics applications, but it can be used in extra contexts.
It is built upon Eigen for linear algebra and FCL for collision detection. **Pinocchio** comes with a Python interface for fast code prototyping.

**Pinocchio** is now at the hearth of various robotics softwares as the [Stack-of-Tasks](http://stack-of-tasks.github.io) or the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc).

## Pinocchio features

**Pinocchio** is fast:

   - C++ template library,
   - cache friendly,
   - automatic code generation support.

**Pinocchio** implements rigid body dynamics algorithms:

   - forward kinematics and its analytical derivatives
   - forward and inverse dynamics,
   - analytical derivatives of forward and inverse dynamics,
   - centroidal dynamics.

**Pinocchio** is multi-thread friendly.  
**Pinocchio** is reliable and extensively tested (unit-tests, simulations and real robotics applications).  
**Pinocchio** is supported on MacOs and many Linux distribution ([See build status here](http://robotpkg.openrobots.org/rbulk/robotpkg/math/pinocchio/index.html)).

## Ongoing developments

If you want to follow the current developments, you can directly refer to the [devel branch](https://github.com/stack-of-tasks/pinocchio/tree/devel).

## Installation

**Pinocchio** can be easily installed on various Linux (Ubuntu, Fedora, etc.) and Unix distributions (Mac OS X, BSD, etc.). Please refer to the [installation procedure](http://stack-of-tasks.github.io/pinocchio/download.html).

## Examples

We provide some basic examples on how to use **Pinocchio** in Python in the [examples/python](./examples/python/README.md) directory.

## Tutorials

**Pinocchio** is coming with a large bunch of tutorials aiming at introducing the basic tools for robotics control.
The content of the tutorials are described [here](http://projects.laas.fr/gepetto/index.php/Teach/Supaero2018) and the source code of these tutorials is located [here](https://github.com/stack-of-tasks/pinocchio-tutorials).

## Citing Pinocchio

To cite **Pinocchio** in your academic research, please use the following bibtex lines:
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
  title = {Analytical Derivatives of Rigid Body Dynamics Algorithms},
  author = {Carpentier, Justin and Mansard, Nicolas},
  booktitle = {Robotics: Science and Systems},
  year = {2018}
}
```

## Questions and Issues

You have a question or an issue? You may either directly open a [new issue](https://github.com/stack-of-tasks/pinocchio/issues) or use the mailing list <pinocchio@laas.fr>.

## Credits

The following people have been involved in the development of **Pinocchio**:

- [Justin Carpentier](https://jcarpent.github.io) (INRIA): main developer and manager of the project
- [Nicolas Mansard](http://projects.laas.fr/gepetto/index.php/Members/NicolasMansard) (LAAS-CNRS): project instructor
- [Guilhem Saurel](http://projects.laas.fr/gepetto/index.php/Members/GuilhemSaurel) (LAAS-CNRS): continuous integration and deployment
- [Joseph Mirabel](http://jmirabel.github.io/) (LAAS-CNRS): Lie groups support
- [Antonio El Khoury](https://www.linkedin.com/in/antonioelkhoury) (Wandercraft): bug fixes
- [Gabriele Buondono](http://projects.laas.fr/gepetto/index.php/Members/GabrieleBuondonno) (LAAS-CNRS): bug fixes
- [Florian Valenza](https://fr.linkedin.com/in/florian-valenza-1b274082) (Astek): core developments and FCL support

If you have taken part to the development of **Pinocchio**, feel free to add your name and contribution here.

## Acknowledgments

The development of **Pinocchio** is supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr) and the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr).
