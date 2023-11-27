<p align="center">
  <img src="https://raw.githubusercontent.com/stack-of-tasks/pinocchio/master/doc/images/pinocchio-logo-large.png" width="800" alt="Pinocchio Logo" align="center"/>
</p>

<p align="center">
  <a href="https://opensource.org/licenses/BSD-2-Clause"><img src="https://img.shields.io/badge/License-BSD%202--Clause-green.svg" alt="License"/></a>
  <a href="https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/"><img src="https://img.shields.io/badge/docs-online-brightgreen" alt="Documentation"/></a>
  <a href="http://projects.laas.fr/gepetto/doc/stack-of-tasks/pinocchio/master/coverage/"><img src="https://gepgitlab.laas.fr/stack-of-tasks/pinocchio/badges/master/coverage.svg?job=doc-coverage" alt="Coverage Report"/></a>
  <a href="https://anaconda.org/conda-forge/pinocchio"><img src="https://img.shields.io/conda/dn/conda-forge/pinocchio.svg" alt="Conda Downloads"/></a>
  <a href="https://anaconda.org/conda-forge/pinocchio"><img src="https://img.shields.io/conda/vn/conda-forge/pinocchio.svg" alt="Conda Version"/></a>
  <a href="https://badge.fury.io/py/pin"><img src="https://badge.fury.io/py/pin.svg" alt="PyPI version" height="20"></a>
  <br>
  <!--<a href="https://gitlab.laas.fr/stack-of-tasks/pinocchio"><img src="https://gitlab.laas.fr/stack-of-tasks/pinocchio/badges/master/pipeline.svg" alt="Pipeline Status"></a>-->

</p>

<!--Pinocchio: a C++ library for efficient Rigid Multi-body Dynamics computations
===========
-->

**Pinocchio** instantiates the state-of-the-art Rigid Body Algorithms for poly-articulated systems based on revisited Roy Featherstone's algorithms.
Besides, **Pinocchio** provides the analytical derivatives of the main Rigid-Body Algorithms like the Recursive Newton-Euler Algorithm or the Articulated-Body Algorithm.

**Pinocchio** was first tailored for robotics applications, but it can be used in other contexts (biomechanics, computer graphics, vision, etc.).
It is built upon Eigen for linear algebra and FCL for collision detection. **Pinocchio** comes with a Python interface for fast code prototyping, [directly accessible](https://github.com/conda-forge/pinocchio-feedstock#installing-pinocchio) through [Conda](https://docs.conda.io/en/latest/).

**Pinocchio** is now at the heart of various robotics software as [Crocoddyl](https://github.com/loco-3d/crocoddyl/tree/devel), an open-source and efficient Differential Dynamic Programming solver for robotics, the [Stack-of-Tasks](http://stack-of-tasks.github.io), an open-source and versatile hierarchical controller framework or the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), open-source software for Motion and Manipulation Planning.

If you want to learn more about **Pinocchio** internal behaviors and main features, we invite you to read the related [paper](https://hal-laas.archives-ouvertes.fr/hal-01866228) and the online [documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/).

If you want to dive into **Pinocchio** directly, only one single line is sufficient (assuming you have Conda):

<p align="center">
<strong> conda install pinocchio -c conda-forge </strong>
</p>

or via pip (currently only available on Linux):
<p align="center">
<strong> pip install pin </strong>
</p>

## Table of contents 

  - [Pinocchio main features](#pinocchio-main-features)
  - [Documentation](#documentation)
  - [Examples](#examples)
  - [Tutorials](#tutorials)
  - [Performances](#performances)
  - [Ongoing developments](#ongoing-developments)
  - [Visualization](#visualization)
  - [Citing Pinocchio](#citing-pinocchio)
  - [Questions and Issues](#questions-and-issues)
  - [Credits](#credits)
  - [Open-source projects relying on Pinocchio](#open-source-projects-relying-on-pinocchio)
  - [Acknowledgments](#acknowledgments)

## Pinocchio main features

**Pinocchio** is fast:

   - C++ template library,
   - cache friendly,
   - automatic code generation support is available via [CppADCodeGen](https://github.com/joaoleal/CppADCodeGen).

**Pinocchio** is versatile, implementing basic and more advanced rigid body dynamics algorithms:

   - forward kinematics and its analytical derivatives,
   - forward/inverse dynamics and their analytical derivatives,
   - centroidal dynamics and its analytical derivatives,
   - support multiple precision arithmetic via Boost.Multiprecision or any similar framework,
   - computations of kinematic and dynamic regressors for system identification and more,
   - and much more with the support of modern and open-source Automatic Differentiation frameworks like [CppAD](https://github.com/coin-or/CppAD) or [CasADi](https://web.casadi.org/).

**Pinocchio** is flexible:

   - header only,
   - C++ 98/03/11/14/17/20 compliant.

**Pinocchio** is extensible.
**Pinocchio** is multi-thread friendly.
**Pinocchio** is reliable and extensively tested (unit-tests, simulations, and real-world robotics applications).
**Pinocchio** is supported and tested on Windows, Mac OS X, Unix, and Linux ([see build status here](http://robotpkg.openrobots.org/rbulk/robotpkg/math/pinocchio/index.html)).

## Documentation

The online **Pinocchio** documentation of the last release is available [here](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/). A cheat sheet pdf with the main functions and algorithms can be found [here](https://github.com/stack-of-tasks/pinocchio/blob/master/doc/pinocchio_cheat_sheet.pdf).

## Examples

We provide some basic examples of using **Pinocchio** in Python in the [examples](https://github.com/stack-of-tasks/pinocchio/tree/master/examples) directory.
Additional examples introducing **Pinocchio** are also available in the [documentation](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_d-practical-exercises_intro.html).

## Tutorials

**Pinocchio** comes with a large bunch of tutorials aiming at introducing the basic tools for robot control.
Tutorial and training documents are listed [here](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/index.html#OverviewConclu).
You can also consider the interactive Jupyter notebook [set of tutorials](https://github.com/ymontmarin/_tps_robotique) developed by [Nicolas Mansard](https://gepettoweb.laas.fr/index.php/Members/NicolasMansard) and [Yann de Mont-Marin](https://github.com/ymontmarin).

## Pinocchio continuous integrations

**Pinocchio** is constantly tested for several platforms and distributions, as reported below:
<p align="center">
  <table class="center">
   <!-- <tr> <td> Continuous Integration </td></tr>-->
  <tr><td> CI on ROS </td>
  <td><a href="https://github.com/stack-of-tasks/pinocchio/actions/workflows/ros-ci.yml"><img alt="ROS" src="https://github.com/stack-of-tasks/pinocchio/actions/workflows/ros-ci.yml/badge.svg?branch=devel" /></a></td>
    </tr><tr><td> CI on Linux via APT </td>
  <td><a href="https://github.com/stack-of-tasks/pinocchio/actions/workflows/linux.yml"><img alt="linux" src="https://github.com/stack-of-tasks/pinocchio/actions/workflows/linux.yml/badge.svg?branch=devel" /></a></td>
    </tr><tr><td> CI on OSX via Conda </td>
  <td><a href="https://github.com/stack-of-tasks/pinocchio/actions/workflows/macos-linux-conda.yml"><img alt="mac" src="https://github.com/stack-of-tasks/pinocchio/actions/workflows/macos-linux-conda.yml/badge.svg?branch=devel" /></a></td>
    </tr><tr><td> CI on Windows via Conda </td>
  <td><a href="https://github.com/stack-of-tasks/pinocchio/actions/workflows/windows-conda-clang.yml"><img alt="windows" src="https://github.com/stack-of-tasks/pinocchio/actions/workflows/windows-conda-clang.yml/badge.svg?branch=devel" /></a></td>
  </tr><tr><td> CI on Linux via Robotpkg </td>
    <td><img src="https://gitlab.laas.fr/stack-of-tasks/pinocchio/badges/master/pipeline.svg" alt="Pipeline Status"></td>
   </tr>
  </table>
</p>

## Performances

**Pinocchio** exploits, at best, the sparsity induced by the kinematic tree of robotics systems. Thanks to modern programming language paradigms, **Pinocchio** can unroll most of the computations directly at compile time, allowing to achieve impressive performances for a large range of robots, as illustrated by the plot below, obtained on a standard laptop equipped with an Intel Core i7 CPU @ 2.4 GHz.

<p align="center">
  <img src="https://raw.githubusercontent.com/stack-of-tasks/pinocchio/master/doc/images/pinocchio-performances.png" width="600" alt="Pinocchio Logo" align="center"/>
</p>

For other benchmarks, and mainly the capacity of Pinocchio to exploit, at best, your CPU capacities using advanced code generation techniques, we refer to the technical [paper](https://hal-laas.archives-ouvertes.fr/hal-01866228).
In addition, the [introspection](https://github.com/rbd-benchmarks/rbd-benchmarks) may also help you to understand and compare the performances of the modern rigid body dynamics libraries.

## Ongoing developments

If you want to follow the current developments, you can directly refer to the [devel branch](https://github.com/stack-of-tasks/pinocchio/tree/devel).
The [master branch](https://github.com/stack-of-tasks/pinocchio/tree/master/) only contains the latest release. Any new Pull Request should then be submitted on the [devel branch](https://github.com/stack-of-tasks/pinocchio/tree/devel/).

## Installation

**Pinocchio** can be easily installed on various Linux (Ubuntu, Fedora, etc.) and Unix distributions (Mac OS X, BSD, etc.). Please refer to the [installation procedure](http://stack-of-tasks.github.io/pinocchio/download.html).

If you only need the Python bindings of Pinocchio, you may prefer to install it through [Conda](https://docs.conda.io/en/latest/). Please follow the procedure described [here](https://github.com/conda-forge/pinocchio-feedstock#installing-pinocchio).

### ROS

**Pinocchio** is also deployed on ROS. You may follow its deployment status below. If you're interested in using Pinocchio on systems and/or with packages that integrate with the ROS ecosystem, we recommend the installation of Pinocchio via the binaries distributed via the ROS PPA. Here, you can install Pinocchio using `sudo apt install ros-$ROS_DISTRO-pinocchio`. This installs Pinocchio with HPP-FCL support and with Python bindings. You can then depend on Pinocchio in your `package.xml` config (`<depend>pinocchio</depend>`) and include it via CMake (`find_package(pinocchio REQUIRED)`) -- we include support and hooks to discover the package for both ROS1 and ROS2. An example can be found [here](https://github.com/wxmerkt/pinocchio_ros_example). Please note that we always advise including the `pinocchio/fwd.hpp` header as the first include to avoid compilation errors from differing Boost-variant sizes.

| ROS1        |                                                                                                                                                                            | &nbsp;&nbsp;&nbsp;&nbsp; | ROS2         |                                                                                                                                                                            |
| ----------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------ | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Melodic** | [![](https://build.ros.org/job/Mbin_uB64__pinocchio__ubuntu_bionic_amd64__binary/badge/icon)](https://build.ros.org/job/Mbin_uB64__pinocchio__ubuntu_bionic_amd64__binary) | &nbsp;&nbsp;&nbsp;&nbsp; | **Foxy**     | [![](https://build.ros2.org/job/Fbin_uF64__pinocchio__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros2.org/job/Fbin_uF64__pinocchio__ubuntu_focal_amd64__binary) |
| **Noetic**  | [![](https://build.ros.org/job/Nbin_uF64__pinocchio__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uF64__pinocchio__ubuntu_focal_amd64__binary)   | &nbsp;&nbsp;&nbsp;&nbsp; | **Galactic** | [![](https://build.ros2.org/job/Gbin_uF64__pinocchio__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros2.org/job/Gbin_uF64__pinocchio__ubuntu_focal_amd64__binary) |
|             |                                                                                                                                                                            | &nbsp;&nbsp;&nbsp;&nbsp; | **Humble**   | [![](https://build.ros2.org/job/Hbin_uJ64__pinocchio__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__pinocchio__ubuntu_jammy_amd64__binary) |
|             |                                                                                                                                                                            | &nbsp;&nbsp;&nbsp;&nbsp; | **Rolling**  | [![](https://build.ros2.org/job/Rbin_uJ64__pinocchio__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__pinocchio__ubuntu_jammy_amd64__binary) |

## Visualization

**Pinocchio** provides support for many open-source and free visualizers:

-   [Gepetto Viewer](https://github.com/Gepetto/gepetto-viewer): a C++ viewer based on [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph) with Python bindings and Blender export. See [here](https://github.com/stack-of-tasks/pinocchio-gepetto-viewer) for a C++ example on mixing **Pinocchio** and **Gepetto Viewer**.
-   [Meshcat](https://github.com/rdeits/meshcat): supporting visualization in Python and which can be embedded inside any browser.
-   [Panda3d](https://github.com/ikalevatykh/panda3d_viewer): supporting visualization in Python and which can be embedded inside any browser.
-   [RViz](https://github.com/ros-visualization/rviz): supporting visualization in Python and which can interact with other ROS packages.

Many external viewers can also be integrated. See the example [here](https://github.com/stack-of-tasks/pinocchio/blob/master/bindings/python/pinocchio/visualize/base_visualizer.py) for more information.

## Citing Pinocchio

To cite **Pinocchio** in your academic research, please use the following bibtex entry:
```bibtex
@inproceedings{carpentier2019pinocchio,
   title={The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives},
   author={Carpentier, Justin and Saurel, Guilhem and Buondonno, Gabriele and Mirabel, Joseph and Lamiraux, Florent and Stasse, Olivier and Mansard, Nicolas},
   booktitle={IEEE International Symposium on System Integrations (SII)},
   year={2019}
}
```
and the following one for the link to the GitHub codebase:
```bibtex
@misc{pinocchioweb,
   author = {Justin Carpentier and Florian Valenza and Nicolas Mansard and others},
   title = {Pinocchio: fast forward and inverse dynamics for poly-articulated systems},
   howpublished = {https://stack-of-tasks.github.io/pinocchio},
   year = {2015--2021}
}
```
The algorithms for the analytical derivatives of rigid-body dynamics algorithms are detailed here:
```bibtex
@inproceedings{carpentier2018analytical,
  title = {Analytical Derivatives of Rigid Body Dynamics Algorithms},
  author = {Carpentier, Justin and Mansard, Nicolas},
  booktitle = {Robotics: Science and Systems},
  year = {2018}
}
```

## Questions and Issues

Do you have a question or an issue? You may either directly open a [new question](https://github.com/stack-of-tasks/pinocchio/discussions/new?category=q-a) or a [new issue](https://github.com/stack-of-tasks/pinocchio/issues) or, directly contact us via the mailing list <pinocchio@inria.fr>.

## Credits

The following people have been involved in the development of **Pinocchio** and are warmly thanked for their contributions:

-   [Justin Carpentier](https://jcarpent.github.io) (Inria): main developer and manager of the project
-   [Nicolas Mansard](http://projects.laas.fr/gepetto/index.php/Members/NicolasMansard) (LAAS-CNRS): initial project instructor
-   [Guilhem Saurel](http://projects.laas.fr/gepetto/index.php/Members/GuilhemSaurel) (LAAS-CNRS): continuous integration and deployment
-   [Joseph Mirabel](http://jmirabel.github.io/) (Eureka Robotics): Lie groups and hpp-fcl implementation
-   [Antonio El Khoury](https://www.linkedin.com/in/antonioelkhoury) (Wandercraft): bug fixes
-   [Gabriele Buondono](http://projects.laas.fr/gepetto/index.php/Members/GabrieleBuondonno) (LAAS-CNRS): features extension, bug fixes, and Python bindings
-   [Florian Valenza](https://fr.linkedin.com/in/florian-valenza-1b274082) (Astek): core developments and hpp-fcl support
-   [Wolfgang Merkt](http://www.wolfgangmerkt.com/) (University of Oxford): ROS integration and support
-   [Rohan Budhiraja](https://scholar.google.com/citations?user=NW9Io9AAAAAJ) (LAAS-CNRS): features extension
-   [Loïc Estève](https://github.com/lesteve) (Inria): Conda integration and support
-   [Igor Kalevatykh](https://github.com/ikalevatykh) (Inria): Panda3d viewer support
-   [Matthieu Vigne](https://github.com/matthieuvigne) (Wandercraft): MeshCat viewer support
-   [Robin Strudel](https://www.di.ens.fr/robin.strudel/) (Inria): features extension
-   [François Keith](https://scholar.google.fr/citations?user=cxSxXiQAAAAJ&hl=en) (CEA): Windows support
-   [Sarah El Kazdadi](https://github.com/sarah-ek) (Inria): multi-precision arithmetic support
-   [Nicolas Torres Alberto](https://scholar.google.com/citations?user=gYNLhEIAAAAJ&hl=en) (Inria): features extension
-   [Etienne Arlaud](https://github.com/EtienneAr) (Inria): RViz viewer support
-   [Wilson Jallet](https://github.com/ManifoldFR) (LAAS-CNRS/Inria): extension of Python bindings
-   [Fabian Schramm](https://github.com/fabinsch) (Inria): core developper
-   [Shubham Singh](https://github.com/shubhamsingh91) (UT Austin): second-order inverse dynamics derivatives
-   [Stéphane Caron](https://scaron.info) (Inria): core developper
-   [Joris Vaillant](https://github.com/jorisv) (Inria): core developer and manager of the project
-   [Sebastian Castro](https://roboticseabass.com) (PickNik Robotics): MeshCat viewer features extension

If you have participated in the development of **Pinocchio**, please add your name and contribution to this list.

## Open-source projects relying on Pinocchio

-   [Crocoddyl](https://github.com/loco-3d/crocoddyl) A software to realize model predictive control for complex robotics platforms.
-   [TSID](https://github.com/stack-of-tasks/tsid/) A software that implements a Task Space Inverse Dynamics QP.
-   [HPP](https://humanoid-path-planner.github.io/hpp-doc/ ) A SDK that implements motion planners for humanoids and other robots.
-   [Jiminy](https://github.com/duburcqa/jiminy) A simulator based on Pinocchio.
-   [ocs2](https://github.com/leggedrobotics/ocs2) A toolbox for Optimal Control for Switched Systems (OCS2)
-   [TriFingerSimulation](https://github.com/open-dynamic-robot-initiative/trifinger_simulation) TriFinger Robot Simulation (a Robot to perform RL on manipulation).
-   [Casadi_Kin_Dyn](https://github.com/ADVRHumanoids/casadi_kin_dyn) IIT Package for generation of symbolic (SX) expressions of robot kinematics and dynamics.

## Acknowledgments

The development of **Pinocchio** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr) and the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr).
