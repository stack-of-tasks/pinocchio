Pinocchio
===========

[![License LGPL 3](https://img.shields.io/badge/license-LGPLv3-green.svg)](http://www.gnu.org/licenses/lgpl-3.0.txt)
[![Build Status](https://travis-ci.org/stack-of-tasks/pinocchio.svg?branch=devel)](https://travis-ci.org/stack-of-tasks/pinocchio)
[![Coverage Status](https://coveralls.io/repos/github/stack-of-tasks/pinocchio/badge.svg?branch=devel)](https://coveralls.io/github/stack-of-tasks/pinocchio?branch=devel)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/7824/badge.svg)](https://scan.coverity.com/projects/pinocchio)

**Warning:** This repository contains [Git
submodules][git-submodules]. Please clone this repository using the
`git clone --recursive` command. If you already have cloned the
repository, you can run `git submodule init && git submodule update`
to retrieve the submodules.


For general information about the project, please refer to its
homepage: http://stack-of-tasks.github.io/pinocchio/

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The Pinocchio software depends on several packages which
have to be available on your machine.

 - Libraries:
   - eigen3 (version >= 3.0.5)
   - boost unit_test_framework filesystem 
   - Optional:
      - urdfdom (version >= 0.3.0)
      - LUA (version == 5.1)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
 - Bindings:
   - Python: Python 2.7 + Numpy + Boost python + EigenPy (https://github.com/stack-of-tasks/eigenpy.git)
 
### Install standalone urdfdom

In order to read urdf files (see http://wiki.ros.org/urdf for the description), one haves to install the urdfdom package which can come either along ROS library or be installed as a standalone library. Next section describes the second procedure.

urdfdom depends on both console_bridge and urdfdom_headers. The installation of both dependencies can be done with the following command lines in a terminal :
  - git clone git://github.com/ros/console_bridge.git && cd console_bridge && mkdir build && cd build && cmake .. && make && sudo make install
  - git clone git://github.com/ros/urdfdom_headers && cd urdfdom_headers && mkdir build && cd build && cmake .. && make && sudo make install
    
Finally, you just need to apply the following command line to install urdfdom library :
  - git clone git://github.com/ros/urdfdom && cd urdfdom && mkdir build && cd build && cmake .. && make && sudo make install
