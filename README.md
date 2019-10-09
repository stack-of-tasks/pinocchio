Pinocchio: a C++ library for efficient Rigid Multi-body Dynamics computations
===========

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Build Status](https://travis-ci.org/stack-of-tasks/pinocchio.svg?branch=master)](https://travis-ci.org/stack-of-tasks/pinocchio)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/pinocchio/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/pinocchio/master/coverage/)
[![Conda Downloads](https://img.shields.io/conda/dn/conda-forge/pinocchio.svg)](https://anaconda.org/conda-forge/pinocchio)
[![Conda Version](https://img.shields.io/conda/vn/conda-forge/pinocchio.svg)](https://anaconda.org/conda-forge/pinocchio)
[![Anaconda-Server Badge](https://anaconda.org/conda-forge/pinocchio/badges/installer/conda.svg)](https://conda.anaconda.org/conda-forge)


**Pinocchio** instantiates the state-of-the-art Rigid Body Algorithms for poly-articulated systems based on revisited Roy Featherstone's algorithms.
Besides, **Pinocchio** provides the analytical derivatives of the main Rigid-Body Algorithms like the Recursive Newton-Euler Algorithm or the Articulated-Body Algorithm.

**Pinocchio** is first tailored for robotics applications, but it can be used in extra contexts (biomechanics, computer graphics, vision, etc.).
It is built upon Eigen for linear algebra and FCL for collision detection. **Pinocchio** comes with a Python interface for fast code prototyping, [directly accessible](https://github.com/conda-forge/pinocchio-feedstock#installing-pinocchio) through [Conda](https://docs.conda.io/en/latest/).

**Pinocchio** is now at the heart of various robotics softwares as the [Stack-of-Tasks](http://stack-of-tasks.github.io) or the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc).

If you want to learn more on **Pinocchio** internal behaviors and main features, we invite you to read the related [paper](https://hal-laas.archives-ouvertes.fr/hal-01866228).

## Pinocchio features

**Pinocchio** is fast:

   - C++ template library,
   - cache friendly,
   - automatic code generation support.

**Pinocchio** is versatile, implementing basic and more advanced rigid body dynamics algorithms:

   - forward kinematics and its analytical derivatives,
   - forward/inverse dynamics and their analytical derivatives,
   - centroidal dynamics and its analytical derivatives.

**Pinocchio** is flexible:

   - header only,
   - C++11/14/17/20 compliant.

**Pinocchio** is multi-thread friendly.  
**Pinocchio** is reliable and extensively tested (unit-tests, simulations and real robotics applications).  
**Pinocchio** is supported on Mac OS X and many Linux distributions ([see build status here](http://robotpkg.openrobots.org/rbulk/robotpkg/math/pinocchio/index.html)).

## Ongoing developments

If you want to follow the current developments, you can directly refer to the [devel branch](https://github.com/stack-of-tasks/pinocchio/tree/devel).

## Installation

**Pinocchio** can be easily installed on various Linux (Ubuntu, Fedora, etc.) and Unix distributions (Mac OS X, BSD, etc.). Please refer to the [installation procedure](http://stack-of-tasks.github.io/pinocchio/download.html).

If you only need the Python bindings of Pinocchio, you may prefer to install it through [Conda](https://docs.conda.io/en/latest/). Please follow the procedure described [here](https://github.com/conda-forge/pinocchio-feedstock#installing-pinocchio).

**Pinocchio** is also being deployed on ROS, you may follow its deployment status on [Melodic](https://index.ros.org/r/pinocchio/#melodic) or [Kinetic](https://index.ros.org/r/pinocchio/#kinetic).

## Documentation

The online **Pinocchio** documentation of the last release is available [here](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/).

## Examples

We provide some basic examples on how to use **Pinocchio** in Python in the [examples/python](./examples/python) directory.
Additional examples introducing **Pinocchio** are also available in the [documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_d-practical-exercises_intro.html)

## Tutorials

**Pinocchio** is coming with a large bunch of tutorials aiming at introducing the basic tools for robotics control.
The content of the tutorials are described [here](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_d-practical-exercises_1-directgeom.html).

## Citing Pinocchio

To cite **Pinocchio** in your academic research, please use the following bibtex lines:
```
@misc{pinocchioweb,
   author = {Justin Carpentier and Florian Valenza and Nicolas Mansard and others},
   title = {Pinocchio: fast forward and inverse dynamics for poly-articulated systems},
   howpublished = {https://stack-of-tasks.github.io/pinocchio},
   year = {2015--2019}
}
```
and the following one for the reference to the paper introducing **Pinocchio**:
```
@inproceedings{carpentier2019pinocchio,
   title={The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives},
   author={Carpentier, Justin and Saurel, Guilhem and Buondonno, Gabriele and Mirabel, Joseph and Lamiraux, Florent and Stasse, Olivier and Mansard, Nicolas},
   booktitle={IEEE International Symposium on System Integrations (SII)},
   year={2019}
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
- [Gabriele Buondono](http://projects.laas.fr/gepetto/index.php/Members/GabrieleBuondonno) (LAAS-CNRS): features extension, bug fixes and Python bindings
- [Florian Valenza](https://fr.linkedin.com/in/florian-valenza-1b274082) (Astek): core developments and FCL support
- [Wolfgang Merkt](http://www.wolfgangmerkt.com/) (University of Edinburgh): ROS integration and support
- [Rohan Budhiraja](https://scholar.google.com/citations?user=NW9Io9AAAAAJ) (LAAS-CNRS): features extension
- [Loïc Estève](https://github.com/lesteve) (INRIA): Conda integration and support

If you have taken part to the development of **Pinocchio**, feel free to add your name and contribution here.

## Acknowledgments

The development of **Pinocchio** is supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr) and the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr).
