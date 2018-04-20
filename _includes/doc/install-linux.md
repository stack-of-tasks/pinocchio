For Linux system, Pinocchio uses the [robotpkg framework](http://robotpkg.openrobots.org) to package the release
versions and to resolve the dependencies.

## What is robotpkg ?

robotpkg is a package manager tailored for robotics softwares. It greatly simplifies the release of new versions along
with the management of their dependencies.

## Ubuntu LTS 12.04, 14.04, 16.04 and later

The software binaries of the packages managed by robotpkg are directly available through the apt repository
of robotpkg for the Ubuntu LTS 12.04, 14.04, 16.04 and later. You just need to add the robotpkg apt repository to your
sources.list and typically use `sudo apt install robotpkg- + packagename` to install a missing software and its
dependencies. Below, we recall the steps to first add the robotpkg apt repository and then install Pinocchio.

### Add robotpkg apt repository

If you have never added robotpkg as a softwares repository, please follow first the instructions from 1 to 4.
Otherwise, go directly to instruction 5. Those instructions are similar to the installation procedures presented in
[http://robotpkg.openrobots.org/debian.html](http://robotpkg.openrobots.org/debian.html).

1. Check your distribution codename in a terminal with the following command:

```
$ lsb_release -c
Codename:       trusty
```

2. Add robotpkg as source repository to apt:

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub trusty robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub trusty robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

You may need to change trusty according to your current distribution codename.

3. Register the authentication certificate of robotpkg:

```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```

4. You need to run at least once apt update to fetch the package descriptions:

```
sudo apt-get update
```

### Install Pinocchio

5. The installation of Pinocchio and its dependencies is made through the line:

```
sudo apt install robotpkg-pinocchio
```

It will install all the systems and additional required dependences.

### Configure environment variables

All the packages will be installed in the `/opt/openrobots` directory. To make use of installed libraries and programs,
you must need to configure your `PATH`, `PKG_CONFIG_PATH`, `PYTHONPATH` and other similar environment variables to
point inside this directory. For instance:

```
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH
```

You may directly add those lines to your `$HOME/.bashrc` for a persistent configuration.

## Other distributions

For the other distributions, the easiest way to get Pinocchio is to install robotpkg and let it manage the Pinocchio
installation (see From robotpkg for the instructions). You can check the compilation status on various distributions
through pinocchio-status.
