---
layout: page
title: Download and install
category: Getting Started
---

The software architecture has been tested entirely on Ubuntu 10.04 LTS, 12.04 LTS, and 14.04 LTS. If you are running other distributions, we expect the library to work correctly but we can not guarantee it.

To install Pinocchio from sources, you will first need to perform installations of other packages themselves dependent on other ones. To facilitate the procedure, you will find in the part "Build From Source" all you need to install Pinocchio.

Pinocchio comes with a python binding that you can use to prototype and that makes the link with
[Gepetto-viewer](https://github.com/humanoid-path-planner/gepetto-viewer) to vizualise your robot (Gepetto viewer is a
simple 3D-vizualisation tool we advice).

# Installation procedure

You can choose either to install a pre-compiled release of Pinocchio or just build it directly from source. Here is a list of supported installation methods:

+ [Linux](#linux)
+ [Mac OS X](#mac-os-x)
+ [Build with _robotpkg_](#build-with-robotpkg)
+ [Build from source](#from-source)

## Linux

For Linux system, Pinocchio uses the _robotpkg_ framework (http://robotpkg.openrobots.org) to package the release versions and to resolve the dependencies.

### What is _robotpkg_ ?

_robotpkg_ is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along with the management of their dependencies.

### Ubuntu 12.04 and 14.04

The software binaries of the packages managed by _robotpkg_ are directly available throw the Personal Package Archive (PPA) of _robotpkg_ for the Ubuntu LTS 12.04 and 14.04. You just need to add the _robotpkg_ PPA to your **sources.list** and typically use `sudo apt-get install robotpkg-` + _packagename_ to install a missing software and its dependencies. Below, we recall the steps to first add the _robotpkg_ PPA and then install Pinocchio.

#### Add _robotpkg_ PPA
If you have never added _robotpkg_ as a softwares repository, please follow first the instructions from 1 to 4. Otherwise, go directly to instruction 5.
Those instructions are similar to the installation procedures presented in http://robotpkg.openrobots.org/debian.html.


1.Check your distribution codename in a terminal with the following command:

    % lsb_release -c
    Codename:       trusty

2.Add _robotpkg_ as source repository to _apt_:

    % sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
    deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub/dists trusty robotpkg
    EOF

You may need to change **trusty** according to your current distribution codename.

3.Register the authentication certificate of _robotpkg_:

    % curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
    sudo apt-key add -

4.You need to run at least once `apt-get update` to fetch the package descriptions:

    sudo apt-get update

#### Install Pinocchio

5.The installation of Pinocchio and its dependencies is made throw the line:

    sudo apt-get install robotpkg-pinocchio

#### Configure environment variables

All the packages will be installed in the /opt/openrobots directory. To make use of installed libraries and programs, you must need to configure your PATH, PKG_CONFIG_PATH, PYTHONPATH and other similar environment variables to point inside this directory. For instance:

    export PATH=/opt/openrobots/bin:$PATH
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
    export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
    export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH

You may directly add those lines to your $HOME/.bashrc for a persistent configuration.

### Ubuntu 16.04

Unfortunately, Ubuntu 16.04 comes with the Eigen development version 3.2.92 which includes several changes in the API and are not yet compatible with the current Pinocchio version. We recommend you to install first the last release version of Eigen (available at http://eigen.tuxfamily.org/index.php?title=Main_Page). And finally follows the standard installation procedure.

### Other distributions

For the other distributions, the easiest way to get Pinocchio is to install _robotpkg_ and let it manage the Pinocchio installation (see [From _robotpkg_](#build-with-robotpkg) for the instructions).

***

## Mac OS X

Currently, we do not yet support an automatic installation from a package manager. This feature will be available soon from the [_brew_](http://brew.sh) package manager.

An alternative solution is to install Pinocchio either with _robotpkg_ (see [From _robotpkg_](#from-robotpkg) section for further details) or directly from sources (see [From source](#from-source) section).


***

## Build with _robotpkg_

_robotpkg_ is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along with the management of their dependencies.

### Installation of _robotpkg_

_robotpkg_ can be easily installed as a package manager on any Unix and BSD platforms. Please follow the installation procedure of robotpkg http://robotpkg.openrobots.org/install.html.

### Clone of _robotpkg-wip_

Pinocchio belongs to the work in progress (WIP) repository of _robotpkg_. Please follow the instruction on how to clone _robotpkg-wip_ into _robotpkg_ http://robotpkg.openrobots.org/robotpkg-wip.html.

### Installation of Pinocchio

Assuming that you have cloned _robotpkg_ in $HOME and _robotpkg-wip_ in $HOME/robotpkg, you can directly move to the wip directory with `cd $HOME/robotpkg/wip`. From here, `make update` will try to build and install the latest release of Pinocchio and its dependencies in **/opt/openrobots**.

The last thing you should do is [configure the environment variables](#configure-environment-variables).


***

## From source

This solution is the best choice if you want to work with the development version of Pinocchio or to contribute to the project with your fixes and improvements.

### Dependencies

Pinocchio has only few required dependencies and some optional ones which add more features, making Pinocchio a versatile framework.

Required dependencies:
+ [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) for linear algebra. Version >= 3.0.5
+ [Boost](http://www.boost.org) with components _filesystem_, _unit_test_framework_ and _system_. Version >= 1.34

Optional dependencies:
+ [Boost](http://www.boost.org) with component _python_. Version >= 1.34. Required for the python bindings.
+ [EigenPy](https://github.com/stack-of-tasks/eigenpy) An efficient binding between Numpy and Eigen using boost::python. Version >= 1.3.0. Required for the python bindings.
+ [FCL](https://github.com/humanoid-path-planner/hpp-fcl) the Fast Collision Library. Version >= 0.4.1. Useful for collision detections.
+ [assimp](https://github.com/assimp/assimp) for the reading of raw mesh files. Version >= 3.0.0.
+ [urdfdom](https://github.com/ros/urdfdom) for the reading of URDF models. Version >= 0.2.10.

### Installation

1.Clone the git repository:

    git clone --recursive https://github.com/stack-of-tasks/pinocchio

2.Switch between the master or the devel branch:

    git checkout master
_or_

    git checkout devel

3.Move to pinocchio, create the build directory and move there:

    cd pinocchio && mkdir build && cd build

4.Source with cmake with Release mode and set your desired installation directory:

    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local

5.Build the application:

    make -j4

6.Install pinocchio:

    make install

### Configure environment variables

All the packages will be installed in the CMAKE_INSTALL_PREFIX path, setting by default to point to /usr/local. To make use of installed libraries and programs, you must need to configure your PATH, PKG_CONFIG_PATH, PYTHONPATH and other similar environment variables to point inside this directory. For instance:

    export PATH=/usr/local/bin:$PATH
    export PKG_CONFIG_PATH =/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    export PYTHONPATH =/usr/local/lib/python2.7/site-packages:$PYTHONPATH

You may directly add those lines to your $HOME/.bashrc for a persistent configuration.
