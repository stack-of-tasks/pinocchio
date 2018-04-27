# Build with robotpkg

robotpkg is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along with the management of their dependencies.

## Installation of robotpkg

robotpkg can be easily installed as a package manager on any Unix and BSD platforms. Please follow [the installation procedure of robotpkg](http://robotpkg.openrobots.org/install.html).

## Installation of Pinocchio

Assuming that you have cloned robotpkg in `$HOME`, you can directly move to the `py-pinocchio` directory with `cd
$HOME/robotpkg/math/py-pinocchio`. From here, `make update` will try to build and install the latest release of Pinocchio and its dependencies in `/opt/openrobots`.

The last thing you should do is configure the environment variables.
