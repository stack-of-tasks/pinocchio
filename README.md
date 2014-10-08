Pinocchio
===========

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
   - urdfdom (version > 0.3.0)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
 - Python 2.7
 - Boost python
 
### Install standalone urdfdom

In order to read urdf files (see http://wiki.ros.org/urdf for the description), one haves to install the urdfdom package which can come either along ROS library or be installed as a standalone library. Next section describes the second procedure.

urdfdom depends on both console_bridge and urdfdom_headers. The installation of both dependencies can be done with the following command in a terminal :
  - git clone git://github.com/ros/console_bridge.git && cd console_bridge && mkdir build && cd build && cmake .. && make && sudo make install
  - git clone git://github.com/ros/urdfdom_headers && cd urdfdom_headers && mkdir build && cd build && cmake .. && make && sudo make install
    
Finally, you just need to apply the following command line to install urdfdom library :
  - git clone git://github.com/ros/urdfdom && cd urdfdom && mkdir build && cd build && cmake .. && make && sudo make install
