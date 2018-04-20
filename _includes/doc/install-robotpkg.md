# Build with robotpkg

robotpkg is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along with the management of their dependencies.

## Installation of robotpkg

robotpkg can be easily installed as a package manager on any Unix and BSD platforms. Please follow [the installation procedure of robotpkg](http://robotpkg.openrobots.org/install.html).

## Clone of robotpkg-wip

Pinocchio belongs to the work in progress (WIP) repository of robotpkg. Please follow [the instruction on how to clone robotpkg-wip into robotpkg](http://robotpkg.openrobots.org/robotpkg-wip.html).

## Installation of Pinocchio

Assuming that you have cloned robotpkg in `$HOME` and robotpkg-wip in `$HOME/robotpkg`, you can directly move to the wip directory with `cd $HOME/robotpkg/wip`. From here, `make update` will try to build and install the latest release of Pinocchio and its dependencies in `/opt/openrobots`.

The last thing you should do is configure the environment variables.
