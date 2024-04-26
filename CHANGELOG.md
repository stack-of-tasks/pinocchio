# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [2.7.1] - 2024-04-26

### Changed
- Modify algorithm that appends a model to another ([#2218](https://github.com/stack-of-tasks/pinocchio/pull/2218))
- Set NOMINMAX as a public definitions on Windows ([#2139](https://github.com/stack-of-tasks/pinocchio/pull/2139))
- Improve documentation of `enum ReferenceFrame` ([#2190](https://github.com/stack-of-tasks/pinocchio/pull/2190))
- Improve documentation of `getJointJacobian` ([#2193](https://github.com/stack-of-tasks/pinocchio/pull/2193)).

### Fixed
- CMake now uses Relative Path instead of Absolute ([#2202](https://github.com/stack-of-tasks/pinocchio/pull/2202))
- Order of frames in `ReducedModel` is now the same as in the full model ([#2160](https://github.com/stack-of-tasks/pinocchio/pull/2160))
- Remove a lot of warnings ([#2139](https://github.com/stack-of-tasks/pinocchio/pull/2139))
- `MeshcatVisualizer` doesn't crash anymore when there is no collision model defined ([#2147](https://github.com/stack-of-tasks/pinocchio/pull/2147))
- Fix MSVC build ([#2155](https://github.com/stack-of-tasks/pinocchio/pull/2155))
- Fix stub generation ([#2166](https://github.com/stack-of-tasks/pinocchio/pull/2166))
- Clean up empty documentation pages and sections ([#2167](https://github.com/stack-of-tasks/pinocchio/pull/2167))
- Fix SO(3) title and cross-section reference in the documentation ([#2210](https://github.com/stack-of-tasks/pinocchio/pull/2210))

### Added
- Add `examples/floating-base-velocity-viewer.py` to visualize floating base velocity ([#2143](https://github.com/stack-of-tasks/pinocchio/pull/2143))
- Add remark to the documentation of `getFrame(Classical)Acceleration` functions ([#2169](https://github.com/stack-of-tasks/pinocchio/pull/2169))
- Allow use of installed jrl-cmakemodules ([#2216](https://github.com/stack-of-tasks/pinocchio/pull/2216))

## [2.7.0] - 2024-01-23

### Added
- Add `GeometryObject::meshMaterial` attribute ([#2084](https://github.com/stack-of-tasks/pinocchio/issues/2084))

### Fixed

- Use bp::ssize_t for recent version of Windows compilers ([#2102](https://github.com/stack-of-tasks/pinocchio/pull/2102))
- Fix missing include for Boost >= 1.83 ([#2103](https://github.com/stack-of-tasks/pinocchio/pull/2103))
- Remove f-strings to fix install with python 2 ([#2110](https://github.com/stack-of-tasks/pinocchio/pull/2110))
- CMake: stop exporting CppAd/cppadcodegen & fetch submodule if not available ([#2112](https://github.com/stack-of-tasks/pinocchio/pull/2112))
- Fix malloc issue in CRBA algo ([#2126](https://github.com/stack-of-tasks/pinocchio/pull/2126))
- Fix build cppad and cppadcg with Boost < 1.77 ([#2132](https://github.com/stack-of-tasks/pinocchio/pull/2132))

## [2.6.21] - 2023-11-27

### Added

- Add inverse dynamics (`rnea`) Python and C++ example ([#2083](https://github.com/stack-of-tasks/pinocchio/pull/2083))
- Add visualization of Frames in MeshCat viewer ([#2098](https://github.com/stack-of-tasks/pinocchio/pull/2098))

### Fixed

- Re-initialize `Ycrb[0]` in `crbaMinimal` ([#2040](https://github.com/stack-of-tasks/pinocchio/pull/2040))
- Fix custom scalar use in `log` function ([#2047](https://github.com/stack-of-tasks/pinocchio/pull/2047))
- Raise exception on wrong input size in `XYZQUATToSE3` Python binding function ([#2073](https://github.com/stack-of-tasks/pinocchio/pull/2073))
- Remove memory leak in `buildGeomFromUrdf` and `buildGeomFromUrdfString` Python binding functions ([#2082]()https://github.com/stack-of-tasks/pinocchio/pull/2082)
- Fix Panda3D viewer examples ([#2087](https://github.com/stack-of-tasks/pinocchio/pull/2087))
- Fix centroidal dynamics derivatives with respect to time ([#2094](https://github.com/stack-of-tasks/pinocchio/pull/2094)))

### Changed

- Rename freeflyer_joint to root_joint in `humanoid` sample model ([#2043](https://github.com/stack-of-tasks/pinocchio/pull/2043))
- CMake minimal version is now 3.10 ([#2055](https://github.com/stack-of-tasks/pinocchio/pull/2055))
- Split headers and sources in different directories to have a more standard C++ project ([#2070](https://github.com/stack-of-tasks/pinocchio/pull/2070))

### Removed

- Remove support to `hpp-fcl` < v2.0.0 ([#2086](https://github.com/stack-of-tasks/pinocchio/pull/2086))

## [2.6.20] - 2023-08-09

### What's Changed
- Fix support of recent versions of Boost for CppAD and CppADCodeGen by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/2000
- build(deps): bump ros-industrial/industrial_ci from afbf77f39db26785371161d5691ab435b31bb3ba to 1e0c5aff1147d50d58bf4185a55ff564c9b6e027 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/2007
- Enabled copy and deepcopy by [@cmastalli](https://github.com/cmastalli) in https://github.com/stack-of-tasks/pinocchio/pull/1882
- build(deps): bump ros-industrial/industrial_ci from afbf77f39db26785371161d5691ab435b31bb3ba to 9f963f67ebb889792175776c5ee00134d7bb569b by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/2013
- Sync submodule cmake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/2029

## [2.6.19] - 2023-06-19

### What's Changed
- Add Motion::toHomogeneousMatrix by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1946
- The insatiable English teacher PR 🧙 by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1957
- require C++14 for Boost >= 1.81 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1949
- CMake: an example require python 3 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1966
- Fix IK example in the documentation by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1963
- Issue templates by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1971
- build(deps): bump ros-industrial/industrial_ci from 4b78602d67127a63dce62926769d9ec4e2ce72e4 to afbf77f39db26785371161d5691ab435b31bb3ba by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1976
- Enhance CMake packaging for Windows by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1984
- Documentation  by [@drewhamiltonasdf](https://github.com/drewhamiltonasdf) in https://github.com/stack-of-tasks/pinocchio/pull/1986
- Add support for ccache on Conda build by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1987
- build(deps): bump ros-industrial/industrial_ci from 4b78602d67127a63dce62926769d9ec4e2ce72e4 to afbf77f39db26785371161d5691ab435b31bb3ba by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1988
- Enhance compatibility with new Python versions by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1996

### New Contributors
- [@drewhamiltonasdf](https://github.com/drewhamiltonasdf) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1986

## [2.6.18] - 2023-04-29

### What's Changed
- Support force in pybind11 by [@cmastalli](https://github.com/cmastalli) in https://github.com/stack-of-tasks/pinocchio/pull/1868
- Fix some Python bindings signatures and add stub generation. by [@duburcqa](https://github.com/duburcqa) in https://github.com/stack-of-tasks/pinocchio/pull/1869
- Fix IK example by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1875
- Remove empty examples by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1878
- build(deps): bump ros-industrial/industrial_ci from 6a8f546cbd31fbd5c9f77e3409265c8b39abc3d6 to 4b78602d67127a63dce62926769d9ec4e2ce72e4 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1890
- add static-contact-dynamics example by [@PepMS](https://github.com/PepMS) in https://github.com/stack-of-tasks/pinocchio/pull/1891
- Update documentation of `JointModel.shortname` in python bindings by [@Danfoa](https://github.com/Danfoa) in https://github.com/stack-of-tasks/pinocchio/pull/1892
- update doc by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1898
- Document that joints need to be added in depth-first order by [@traversaro](https://github.com/traversaro) in https://github.com/stack-of-tasks/pinocchio/pull/1899
- fix INSTALL_RPATH on ROS & OSX by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1908
- Python example: update joint placements after loading a URDF by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1911
- Remove more empty sections from the docs by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1912
- Sync submodule cmake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1914
- update doc for fixed joint by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1920
- doc: more information about frames on cheatsheet by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1930
- Define operational frames in the documentation by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1929
- Remove support of np.matrix by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1941
- Fix other np.matrix issues by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1942

### New Contributors
- [@PepMS](https://github.com/PepMS) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1891
- [@Danfoa](https://github.com/Danfoa) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1892

## [2.6.17] - 2023-02-15

### What's Changed
- Fix Jlog6 documentation by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1857
- Add documentation to Jlog3 by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1858
- [python/visualize] Extend visualizer features and implement them for MeshcatVisualizer by [@ManifoldFR](https://github.com/ManifoldFR) in https://github.com/stack-of-tasks/pinocchio/pull/1845
- Second-order RNEA derivatives by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1860
- Sync submodule CMake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1863
- Fix support of AVX2. by [@duburcqa](https://github.com/duburcqa) in https://github.com/stack-of-tasks/pinocchio/pull/1865

## [2.6.16] - 2023-02-02

### What's Changed
- Enforce testing of Python bindings by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1853
- Fix issue with Python 3.6 by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1855

## [2.6.15] - 2023-01-31

### What's Changed
- More documentation for getJointJacobian by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1836
- Error when integrating SE3 with the same vector given as input and output by [@duburcqa](https://github.com/duburcqa) in https://github.com/stack-of-tasks/pinocchio/pull/1775
- Documentation for Jlog6 by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1842
- remove useless header by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1846
- Fix issue with recent change on master branch for ROS-CI by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1847
- Fix issue with old version of Boost and eigenpy >= 2.9.0 by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1851

## [2.6.14] - 2023-01-13

### What's Changed
- Fix registration of ptr to Python by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1832

## [2.6.13] - 2023-01-12

### What's Changed
- build(deps): bump goanpeca/setup-miniconda from 1 to 2 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1790
- Skip collision pairs between geom on same joint in appendGeometryModel by [@jmirabel](https://github.com/jmirabel) in https://github.com/stack-of-tasks/pinocchio/pull/1791
- Fix issue with Boost 1.78 on OSX systems by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1792
- build(deps): bump goanpeca/setup-miniconda from 1 to 2 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1793
- Add helper functions for computing supported inertia and force by frames by [@EtienneAr](https://github.com/EtienneAr) in https://github.com/stack-of-tasks/pinocchio/pull/1796
- Follow-up : Supported inertia by frame by [@EtienneAr](https://github.com/EtienneAr) in https://github.com/stack-of-tasks/pinocchio/pull/1797
- build(deps): bump goanpeca/setup-miniconda from 1 to 2 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1799
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1801
- build(deps): bump goanpeca/setup-miniconda from 1 to 2 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1802
- build(deps): bump goanpeca/setup-miniconda from 1 to 2 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1809
- Sync submodule cmake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1811
- Fix Python issues by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1821
- Allow to use std::shared_ptr by [@florent-lamiraux](https://github.com/florent-lamiraux) in https://github.com/stack-of-tasks/pinocchio/pull/1822

## [2.6.12] - 2022-11-06

### What's Changed
- Elevate check for ambiguous input argument to an exception by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1781
- Fix packaging issues + sync submodule CMake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1782
- Fix issue with clang by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1784

## [2.6.11] - 2022-10-25

### What's Changed
- URDF: fix loading relative mesh path in urdf by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1748
- Sync submodule cmake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1750
- Add [@note](https://github.com/note) to difference documentation by [@stephane-caron](https://github.com/stephane-caron) in https://github.com/stack-of-tasks/pinocchio/pull/1753
- Configuration limits for joints and model by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1756
- Address remaining warnings + speed-up build for BUILD_TESTING=OFF by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1764
- Add status of ROS builds to README by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1770
- pinocchio/visualize: Add support for cone shape in meshcat dispaly by [@whtqh](https://github.com/whtqh) in https://github.com/stack-of-tasks/pinocchio/pull/1769
- Updates README by [@nikoandpiko](https://github.com/nikoandpiko) in https://github.com/stack-of-tasks/pinocchio/pull/1776
- Enhance cmake packaging by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1777

### New Contributors
- [@stephane-caron](https://github.com/stephane-caron) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1753
- [@whtqh](https://github.com/whtqh) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1769
- [@nikoandpiko](https://github.com/nikoandpiko) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1776

## [2.6.10] - 2022-09-14

### What's Changed
- fix syntax for python 2 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1734
- Expose `removeGeometryObject` in python binding by [@Jiayuan-Gu](https://github.com/Jiayuan-Gu) in https://github.com/stack-of-tasks/pinocchio/pull/1736
- Fix relocalable by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1738
- Fix relative path in urdf by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1742
- Sync submodule CMake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1744

### New Contributors
- [@Jiayuan-Gu](https://github.com/Jiayuan-Gu) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1736

## [2.6.9] - 2022-08-12

### What's Changed
- build(deps): bump actions/checkout from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1669
- build(deps): bump actions/cache from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1670
- Sync example-robot-data by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1671
- update to pybind11 v2.9.2 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1674
- build(deps): bump actions/checkout from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1678
- build(deps): bump actions/cache from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1677
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1679
- build(deps): bump actions/checkout from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1692
- build(deps): bump actions/cache from 2 to 3 by [@dependabot](https://github.com/dependabot) in https://github.com/stack-of-tasks/pinocchio/pull/1691
- Fix bug in appendModel by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1693
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1695
- Fix geometry color default value by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1699
- Add python example appending Urdf and another model by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1698
- SO3 diff impl use quaternion instead of rotation matrix by [@Toefinder](https://github.com/Toefinder) in https://github.com/stack-of-tasks/pinocchio/pull/1702
- add cheat sheet to doc by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1705
- cmake/utils: no need for python here by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1709
- Add header only target by [@fabinsch](https://github.com/fabinsch) in https://github.com/stack-of-tasks/pinocchio/pull/1712
- cmake: relocatable package for recent CMake versions by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1716
- cmake: modernize header-only lib by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1718
- ROS2/Colcon integration by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1719
- CMake: update to eigenpy 2.7.10 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1720
- Fix weird test failure on Conda by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1723
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1727
- ROS2 release support by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1681
- Sync submodule CMake by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1732

### New Contributors
- [@fabinsch](https://github.com/fabinsch) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1698

## [2.6.8] - 2022-06-06

### What's Changed
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1648
- Fix integrate method for SO2 by [@Toefinder](https://github.com/Toefinder) in https://github.com/stack-of-tasks/pinocchio/pull/1652
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1653
- [pre-commit.ci] pre-commit autoupdate by [@pre-commit-ci](https://github.com/pre-commit-ci) in https://github.com/stack-of-tasks/pinocchio/pull/1658
- Extend current Coriolis computations to account for Cristoffel symbol of first kind by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1665
- Fix method play() of rviz_visualizer by [@danielcostanzi18](https://github.com/danielcostanzi18) in https://github.com/stack-of-tasks/pinocchio/pull/1667
- chore: Included githubactions in the dependabot config by [@nathannaveen](https://github.com/nathannaveen) in https://github.com/stack-of-tasks/pinocchio/pull/1659
- Enable ubuntu 22.04 on CI by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1668

### New Contributors
- [@pre-commit-ci](https://github.com/pre-commit-ci) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1648
- [@Toefinder](https://github.com/Toefinder) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1652
- [@danielcostanzi18](https://github.com/danielcostanzi18) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1667
- [@nathannaveen](https://github.com/nathannaveen) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1659

## [2.6.7] - 2022-05-03

### What's Changed
- add dummy .pre-commit-config.yaml by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1629
- Avoid triggering multi-line comment in Latex formula by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1630
- Make Pinocchio v2 compatible with HPP-FCL v2 by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1631
- Add example of collision with a point cloud by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1636
- follow up on hpp-fcl v2 by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1637
- Fix pickling for Boost >= 1.7.4 by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1640

## [2.6.6] - 2022-03-22

### What's Changed
- Support HPP-FCL for ROS binaries & introduce ROS2 ament integration by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1622

## [2.6.5] - 2022-02-14

### What's Changed
- Fix warning issue in Python by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1546
- RVizVisualizer update by [@EtienneAr](https://github.com/EtienneAr) in https://github.com/stack-of-tasks/pinocchio/pull/1549
- Correct link to examples. Removed python subdirectory in the link as the directory does not exist anymore by [@olivier-stasse](https://github.com/olivier-stasse) in https://github.com/stack-of-tasks/pinocchio/pull/1561
- Add CONTRIBUTING.md file for newcomers by [@olivier-stasse](https://github.com/olivier-stasse) in https://github.com/stack-of-tasks/pinocchio/pull/1563
- Add list of projects based on Pinocchio by [@olivier-stasse](https://github.com/olivier-stasse) in https://github.com/stack-of-tasks/pinocchio/pull/1566
- Fix constrained dynamics formula by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1567
- Append "/share" to AMENT_PREFIX_PATH by [@proyan](https://github.com/proyan) in https://github.com/stack-of-tasks/pinocchio/pull/1568
- Expose dIntegrateTransport by [@ManifoldFR](https://github.com/ManifoldFR) in https://github.com/stack-of-tasks/pinocchio/pull/1572
- [timings] Reduce allocations in finite-difference baselines by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1583
- [computeAllTerms] Add missing noalias to avoid temporary allocation by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1582
- Missing `noalias` by [@duburcqa](https://github.com/duburcqa) in https://github.com/stack-of-tasks/pinocchio/pull/1585
- [multibody/model] add joint arg validation by [@proyan](https://github.com/proyan) in https://github.com/stack-of-tasks/pinocchio/pull/1586
- [multibody/geometry] Add method to remove an object. by [@florent-lamiraux](https://github.com/florent-lamiraux) in https://github.com/stack-of-tasks/pinocchio/pull/1588
- Fix bug in ABAChecker by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1599
- Update README.md by [@jmirabel](https://github.com/jmirabel) in https://github.com/stack-of-tasks/pinocchio/pull/1604
- allow `robot_wrapper` to share data with its `viz` by [@vnghia](https://github.com/vnghia) in https://github.com/stack-of-tasks/pinocchio/pull/1606

### New Contributors
- [@duburcqa](https://github.com/duburcqa) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1585
- [@vnghia](https://github.com/vnghia) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1606

## [2.6.4] - 2021-11-02

### What's Changed
- Change 'typedef' to 'using', add curly braces by [@the-raspberry-pi-guy](https://github.com/the-raspberry-pi-guy) in https://github.com/stack-of-tasks/pinocchio/pull/1476
- [CMake] set INSTALL_RPATH for python on linux by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1482
- [visualize] captureImage() method by [@ManifoldFR](https://github.com/ManifoldFR) in https://github.com/stack-of-tasks/pinocchio/pull/1480
- ci: update ROS CI by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1483
- Fix issue with Pool when FCL is missing by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1485
- fix module name by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1487
- Add RVizVisualizer by [@EtienneAr](https://github.com/EtienneAr) in https://github.com/stack-of-tasks/pinocchio/pull/1488
- fix reshape issues by [@kozakromch](https://github.com/kozakromch) in https://github.com/stack-of-tasks/pinocchio/pull/1489
- Remove useless reference to L-GPL + remove of useless files by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1500
- Update old function, remove mobile robot wrapper by [@kozakromch](https://github.com/kozakromch) in https://github.com/stack-of-tasks/pinocchio/pull/1490
- Allows the display of {COLLISION,VISUAL} in MeshCat by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1502
- Delete submodule travis by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1509
- Add citation by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1511
- badges: add PyPI by [@nim65s](https://github.com/nim65s) in https://github.com/stack-of-tasks/pinocchio/pull/1510
- Add support of Convex within MeshCat + improve GeometryObject bindings by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1530
- [python] Fix BaseVisualizer constructor. by [@jmirabel](https://github.com/jmirabel) in https://github.com/stack-of-tasks/pinocchio/pull/1533
- [python] Add pybind11 header. by [@jmirabel](https://github.com/jmirabel) in https://github.com/stack-of-tasks/pinocchio/pull/1519
- make copy of supports during model cast by [@rubengrandia](https://github.com/rubengrandia) in https://github.com/stack-of-tasks/pinocchio/pull/1536
- [python] Simplify buildGeomFromUrdf and allow to build from string. by [@jmirabel](https://github.com/jmirabel) in https://github.com/stack-of-tasks/pinocchio/pull/1538
- Sync submodule by [@jcarpent](https://github.com/jcarpent) in https://github.com/stack-of-tasks/pinocchio/pull/1542
- Turn off automatic documentation generation by [@wxmerkt](https://github.com/wxmerkt) in https://github.com/stack-of-tasks/pinocchio/pull/1541

### New Contributors
- [@the-raspberry-pi-guy](https://github.com/the-raspberry-pi-guy) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1476
- [@ManifoldFR](https://github.com/ManifoldFR) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1480
- [@EtienneAr](https://github.com/EtienneAr) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1488
- [@kozakromch](https://github.com/kozakromch) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1489
- [@rubengrandia](https://github.com/rubengrandia) made their first contribution in https://github.com/stack-of-tasks/pinocchio/pull/1536

## [2.6.3] - 2021-07-27

This new release fixes bugs in MeshCat rendering, loading of ROS path and in the Centroidal dynamics derivatives

## [2.6.2] - 2021-07-05

This new release provides fixes include orders in the unit tests due to recent versions of Boost.
It also provides more content to the Readme.
This new release prefigures the new Pinocchio 3.x

## [2.6.1] - 2021-06-08

Enhancement:
- extended support of serialization of FCL primitives
- extended support of Boost.Variant in Python

Fixes:
- fix bug when loading URDF on Windows
- fix handling of Frame inertia

## [2.6.0] - 2021-04-11

This new release provides:
- improvement for handling collision/distance requests
- support of parallel computations for ABA, RNEA and collisions algorithms (more to come soon)
- additional features for the RobotWrapper in Python
- support of Capsule for Meshcat
- happened Inertia information to Frames
- fixes for doc issues
- improve numerical robustness when two inertias are Zero

## [2.5.6] - 2021-01-23

This new release comes with a more consistent ABA algorithm (data.a_gf used instead of classic data.a) and more outcomes for computeAllTerms. In addition, it provides some extended supports to PyPy.

## [2.5.5] - 2021-01-07

This maintenance release enhances the whole CMake packaging of the project and provides additional features for the Python bindings.

## [2.5.4] - 2020-11-26

This new release enhances the compatibility of Pinocchio with former distributions, provides better support to Euler angles and fixes some bugs.

## [2.5.3] - 2020-11-13

This new release fixes some issues with the Python bindings, add new examples and extend the whole tests.

## [2.5.2] - 2020-11-02

This new release enhances the compatibility with Windows, provides new tools eitheir to work on joint configurations and also to compute the kinematic regressors.

## [2.5.1] - 2020-10-12

This new release provides:
- improvements on the whole project coverage
- better support of Windows v142
- support of joint friction and damping (only at the interface level)
- new algorithm to retrieve the Coriolis matrix from RNEA/ABA derivatives


## [2.5.0] - 2020-08-31

In this new release, we have:
- a full memory shared in the Python bindings, i.e. that all the Eigen object can now be changed in Python.
- better handling of multiprecision arithmetic
- improved CMake packaging
- fix for Autodiff

## [2.4.7] - 2020-07-20

This new release provides:
- an extended API for dealing with Lie groups and the related computations (integration, difference, derivatives, etc.)
- the Lie groups have now their Python bindings for easy code prototyping
- the kinematics derivatives of the Frames have been introduced as well as the extension of current Joints derivatives
- the packaging of the project has been improved too.

## [2.4.6] - 2020-06-15

This new release of Pinocchio removes the use of pkg-config to check dependencies and provides a full compatibility and support for Windows systems.

## [2.4.5] - 2020-05-23

This new release extends the current frames algorithms to also consider the LOCAL_WORLD_ALIGNED reference frame.
It also exposes the values of the enum_ for simplified usage.

## [2.4.4] - 2020-05-13

This new release provides:
- new helpers function to retrieve frame velocities and accelerations
- for each collision pair, it is now possible to provide specific collision checking settings
- support to display convex hulls in viewers

It also fixes:
- the support of Panda3d

and enhances the global CMake packaging of the project.

## [2.4.3] - 2020-04-24

This new release provides a new Viewer, named Panda3d, for easy code visualization in Python.
It also provides additional fixes to the Code Generation support.

## [2.4.2] - 2020-04-21

This new release of Pinocchio provides new features for dealing with Lie groups:
- improved operations over the differential operations of the integrate function
- new functions for transporting some matrix between the two endpoints of the integrate function
- the RPY functions are now robust over singularities
- the support of autodiff frameworks has been improved to also cope with Lie groups features

This release also provides:
- a packaging fixe with respect to the URDFDOM dependency
- the Python bindings should now be without memory leak

## [2.4.1] - 2020-04-15

This new release fixes a bug introduced in Pinocchio 2.4.0 concerning the method ModelTpl::addFrame.
This release also provides better support for the checking of the CppAD and CppADCodeGen versions.

## [2.4.0] - 2020-04-09

This new release of Pinocchio makes several improvements:
- improve compatibility with hpp-fcl
- improve compatibility with CppAD and notably the Lie algebra features
- a better CMake >= 3.0 export of the project
- new examples for Code generation
- improved Python bindings
- improved support for Boost.Multiprecision
- reduce the memory usage when compilation unit tests

and we have started to move some dependencies like urdfdom to the pinocchio.so library to avoid additional compilations issues and to useless compilations burden.

## [2.3.1] - 2020-02-20

This new release provides:
- some fixes with respect to minor bugs introduced in Pinocchio 2.3.0
- an enhance detection of Python
- improves the compilation memory overhead

## [2.3.0] - 2020-02-18

This new release provides:
- full compatibility with CMake export
- full compatibility with Numpy.Array
- examples for Code generation
- better support of C++11
- minor bug fixes
- improves coverage
- uniformizes function signature
- improves interoperability between Numpy and Pinocchio
- add many examples
- full integration of Python bindings of hpp-fcl
- supports the pickling and the serialization of Data

## [2.2.3] - 2019-12-30

This new release provides:
- pickling and serialization of Data structures
- provide new algorithms to create reduce models

It also improves the compatibility with the Transform used in HPP-FCL.
It also fixes various bugs.
It also comes with new and detailed examples.

## [2.2.2] - 2019-12-11

This new release:
- improves the compatibility with HPP-FCL bindings
- improves the documentation of the project with more examples
- fixes some bugs related to Eigen
- add new algorithms to compute the centroidal matrix and its time derivatives

## [2.2.1] - 2019-11-25

This new release fixes the ROS package version number and updates the robot models.

## [2.2.0] - 2019-11-25

This new release of Pinocchio introduces:
- analytical formula for Hessian of the kinematics
- derivatives of the difference operation
- new derivatives for static torque quantity
- new models for tests and examples.

It adds some signature non-exposed in Python.
It also deletes outdated function signatures that have been deprecated in 2.0.x versions.
It also improves the packaging with respect to ROS or other robotics frameworks.

## [2.1.11] - 2019-10-27

This release fixes missing update of the ROS package.xml file with the new version.

## [2.1.10] - 2019-10-25

This new release allows throwing when some input arguments are not fulfilled (useful feature in Python). It also provides new support for Hessian of the kinematics. Finally, the project is now packaged for ROS integration.

## [2.1.9] - 2019-10-09

This is a maintenance release, with some fixes concerning the contact dynamics, better support of LOCAL_WORLD_ALIGNED option and fixes with respect to some recent versions of CppADCodeGen.

## [2.1.8] - 2019-09-30

This is a maintenance release providing several fixes:
- remove memory allocation in ABA derivatives with contact forces
- better handling of boost::Variant
- better support of Majax

It also provides new features in the documentation of mathematical formula.


## [2.1.7] - 2019-09-10

This new release improves:
- the support of AutoDiff frameworks
- the efficiency of some core algorithms

fixes:
- the support of Majax
- the compatibility with Python 2/3

## [2.1.6] - 2019-08-05

This new release improves the packaging of the project and provides new algorithms to compute the Jacobians of the center of mass of each subtree.

## [2.1.5] - 2019-07-16

This new release provides support for JointMimic and JointRevoluteUnboundedUnaligned.
It also comes with the full support of CasADi.

Thanks to [@mkatliar](https://github.com/mkatliar) for helping us to provide this support.

## [2.1.4] - 2019-06-22

This new release provides some fixes with respect to Python bindings, C++17 as well as new important features:

- dynamic regressor for identification
- add support of multiple viewers
- improve analytical derivatives

## [2.1.3] - 2019-04-30

This new release fixes some issues with Python 3 and C++17 standard.
It also provides additional documentation, enlarges the current Python bindings and uniformizes naming convention in Python.

## [2.1.2] - 2019-04-05

This new release aims at fixing compilations issues when COLLISION module is activated.
It also provides some fixes concerning the loading of meshes.

A new feature provided by this release concerns the possibility of appending two models together.

## [2.1.1] - 2019-03-27

This new release fixes compatibility bugs with previous release 2.1.0.
It also provides a serialization interface for Spatial classes and the Model class.

## [2.1.0] - 2019-02-27

This new release makes some major improvements:
- it is now possible to use MeshCat, another viewer working in the browser directly in Python
- the Python bindings are now hardly tested and uniformized with respect to the C++ API
- this new release is compatible with recent releases of `hpp-fcl`
- the SRDF parsing has been improved. Its now possible to load several reference configuration vectors

We also fixed bugs related to recent versions of Boost mostly.

## [2.0.0] - 2019-01-11

Welcome Pinocchio 2.0.0.

This release makes official the last important and new features for efficiently computing the dynamics of the rigid body systems:

- Analytical derivatives
- Automatic differentiation
- Full scalar type overloading
- Code generation among others

## [1.3.3] - 2018-10-29

This is for real the last release before Pinocchio 2.0.0 and more.

This release fixes the packaging when hpp-fcl is missing.
Thanks to [@aelkhour](https://github.com/aelkhour) for raising this issue.

## [1.3.2] - 2018-10-26

This release is the last one before Pinocchio 2.0.0.

It mostly:
- fixes issues introduced by new API of frame functionalities;
- introduces new sample models for manipulator and humanoid systems;
- fixes bugs due to boost 1.58.0;
- improve the readme with credits section.


## [1.3.1] - 2018-09-25

This new release corrects some bugs or bad deprecations concerning Pinocchio 1.3.0.

## [1.3.0] - 2018-08-28

This new release introduces analytical derivatives in the corpus of Pinocchio.
This feature is still under development but can already be used both in C++ and Python.

This new release also fixes a bunch of bugs related to Eigen and Boost.

## [1.2.9] - 2018-06-01

This is mostly a maintenance release:

- Fix bug in. lower bounds in Model class
- Update documentation structure (additional work is needed)
- Improve the compatibility with Python 3.x

## [1.2.8] - 2018-05-18

This is mostly a maintenance release:

- Fix some bugs in JointModel{Translation,Spherical} for ABA algorithm
- Fix a duplication issue in RobotWrapper
- Improve compatibility with recent version of Boost >= 1.67.0
- Romeo is now loaded from the official romeo_description repository



## [1.2.7] - 2018-04-03

This is mostly a maintenance release:

- Fixes and computation improvements for Lie group operations.
- Adding pickle for spatial classes.
- Allow loading of URDF tree directly from an XML stream.

## [1.2.6] - 2018-01-15

This is mostly a maintenance release with various fixes to comply with Boost variadic macro on recent OS.
It also adds new convention with a LOCAL and  a WORLD frame to express Jacobian quantities.

## [1.2.5] - 2017-10-10

This a maintenance Release. We added some algo to compute the time variation of the Jacobians together with the variation with respect to time of the centroidal momemtum matrix.
HPP-FCL works now with Eigen for linear algebra.

## [1.2.4] - 2017-06-09

This is mostly a maintenance release, with some fix with respect new `urdfdom` versions, it handles Eigen support with `hpp-fcl`.

## [1.2.3] - 2017-02-14

This release fixes some issues with respect to 1.2.1.

### API modifications

Interpolate, Differentiate, Integrate are now algorithmic struct which can be efficiently overloaded.

### Bindings

Add FCL object bindings


## [1.2.1] - 2016-10-17

### Summary

This release is a minor patch of the previous release 1.2.0.

This release is directly accessible as a Debian package. Please see https://github.com/stack-of-tasks/pinocchio/wiki/installation for further details.

### New
- The Python bindings are aligned and free of unnecessary allocations
- Add documentation option which allows to not install the documentation
- Introduces container::aligned_vector to automatically create an std::vector with specific aligned allocator

### API modifications
- Remove JointDense.
- Remove JointGeneric

### Fixes
- Solves the parsing of geometries in URDF module
- Fixes alignment issues on 32 bits architecture


## [1.2.0] - 2016-09-29

### Summary

The main modifications concern the update of the code to comply with the Humanoid Path Planner (HPP).

This release is directly accessible as a Debian package. Please see https://github.com/stack-of-tasks/pinocchio/wiki/installation for further details.

### New Features
- Add Joint{Model,Data} classes based on Joint{Model,Data}Base and Joint{Model,Data}Variant. Those classes call directly the visitors and make Variant accessible throw methods
- Add partial Joint{Model,Data}Composite. They allow a stack of joints without adding any Inertia.
- Increase Frame class. Frames can be of several types (BODY, JOINT, SENSOR, etc) and reflect the robot tree as it appears in the URDF conventions. Frames have two attributes: parent which the direct Joint parent in the Joint tree, previousFrame which correspond to the parent Frame in the tree of Frames).
- Adding Python parser which is able to read models written in Python.
- Add algo checker to check the validity of a model.
- Improve documentation.

### API modifications
- Model has some methods deprecated. The default name has been removed.
- The Geometry classes have been updated and several methods have been set to deprecated. They now use Frames as parent instead of Joint directly: a Geometry is now supported by a BODY.
- Add active collision pair flags in GeomData which define the active collision pairs.
- Unify naming conventions (nframes, njoints, etc).

### Fixes
- The UDRF parser can now deal with more complex topologies. It properly handles the stack of geometries for each BODY.
- Improve packaging mainly around the Python part.


## [1.1.2] - 2016-05-31

### Summary

This release is directly accessible as a Debian package. Please see https://github.com/stack-of-tasks/pinocchio/wiki/installation for further details.

### New Features
- Added operational frames (that are a Plucker coordinate frame attached to a parent joint inside a kinematic tree). Position and Jacobian of such frames can be computed
- Geometry primitives can now be handled (and added to the Pinocchio GeometryModel) when encountered in an urdf file
- Implemented simple srdf parsing for GeometryData : parse the desactivated collision pairs
- Added Articulated Body Algorithm (ABA), and CCRBA
- When parsing a urdf file, now look in the environment variable ROS_PACKAGE_PATH for directories where to search for meshes. Users can provide hint directories to search in as a priority. Updated python RobotWrapper consequently
- Added forward dynamics with contact algorithm
- Added algorithms working with vectors of configuration or velocity(either on a JointModel or on a Model, iterating through all the kinematic tree)
  - One can integrate a configuration at a constant velocity during a unit time
  - One can differentiate two configurations (i.e compute the velocity that must be integrated during a unit time to go from first configuration to the other )
  - One can interpolate between two configurations
  - One cancompute the distance between two configurations ( such as dist = norm ( difference) )
  - One can shoot a configuration uniformly sampled between specified limits
- Added Impulse Dynamic Algorithm
- Completed the list of method to access or call Joint's data or method when joint are stored in a variant.
- Added JointAccessor that is a general joint encapsulating a JointVariant ( abstracting the use of visitors for the user).

### API modifications
- Moved limits from joint models to Model as vectors of size nq for position limits and size nv for velocity and effort limits
- The Geometry objects stored in GeometryModel are now splitted in two types : visual and collision

### Minor
- Improved documentation of Data, Model, Spatial Classes
- Improved efficiency when executing algorithms
- One can now create Inertia for simple shapes such as cylinders, boxes, ellipsoid
- Rework some unittests to increase to coverage of whole package ( C++ and Python )

### Bugs Fixed
- Fixed operator Inertia x constraint in JointSphericalZYX
- Fixed the Dense conversion of joints (models and datas)
- Fixed bugs in JointRevoluteUnaligned and JointPrismaticUnaligned to access the access when visiting a variant containing such joints with boost::fusion


## [1.1.0] - 2016-02-04

### Summary

### New Features
- Spatial classes now follow the CRTP Design Pattern, for performance reasons.
- JointModels are now exposed in Python. This feature enables one to load a URDF model with a precise root joint and to create his/her own model.
- Python models can now be created by hand ( ex: buildEmptyModel() + calls to addBody() )
- Added utility tools to check an urdf model ( same as check_urdf but dislpay the Pinocchio model created from urdf parsing)
- Add unaligned prismatic joint
- Add geometry through Flexible Collision Library ([hpp-fcl](https://github.com/humanoid-path-planner/hpp-fcl))
  - Add dedicated structs  to handle geometry. GeometryModel (list of geometry objects and its relation wrt kinematic model) and GeometryData
  - When parsing urdf, meshes can be read from collada files and handled in Pinocchio
  - Created parser that handles geometry and exposed it in python
  - Add algorithms to update the geometry kinematics, to compute the distances for pairs of collision or if they are colliding or not.
- Added algorithms to compute the kinetic energy, the potential energy and exposed it in python
- The complete documentation is in progress

### API modifications
- Change name of kinematics algorithms: now forwardKinematics instead of previous geometry, kinematics and dynamics.

### Minor
- SimpleHumanoid is now built with joint limits
- Handle floating joints in urdf parsing
- Slight separation between joints and body in Model to avoid confusion
- Internally, Motion and Force classes now use a 6D-vector instead of two 3D-vectors for linear and angular part

### Bugs Fixed
- Fixed a bug when trying to merge a link with its parent in case of fixed joint. Now merge only if it has an inertial tag.
- All the algorithms are now set to inline
- Fix compilation errors

### Installation

The source of the release are available in the file **pinocchio-1.1.0.tar.gz** just below along with a binary version for 64-bits Debian architecture of Pinocchio **pinocchio_1.1.0_amd64.deb** and its dependencies. For information, those packages will be installed in _/opt/openrobots_ directory.


## [1.0.2] - 2015-09-14

### Summary

### New Features
- The limits in position, velocity and torque for joints Revolute and Prismatic are now parsed from urdf model and accessible
- Implementation of exp and log functions on SE3 in C++ with its python binding thanks to [@aelkhour](https://github.com/aelkhour)
- Data now contains information relative to the center of mass position, velocity and acceleration
- Add Lua parser - compatible with RBDL
- Add translational joint
- Add planar joint

### Minor
- Reduction of compilation warnings.

### Bugs Fixed
- Fixed bug in operator Y*S in JointRevoluteUnaligned


## [1.0.0] - 2015-04-03

The following algorithms are implemented.
    • Recursive Newton-Euler algorithm (RNEA, i.e inverse dynamics)
    • Composite Rigid Body algorithm (CRBA, i.e generalized inertia matrix)
    • Sparse Cholesky decomposition of the inertia matrix (for constrained forward-dynamics resolution)
    • Placement Jacobians (i.e application from configuration velocities to end-effector spatial velocities), along with computation of body placements, velocities and accelerations.
    • Center of mass and its Jacobian

The model can either be parsed from a URDF format or be created by appendending bodies. The following joint models are implemented.
    • Revolute X, Y, Z (optimized) and unaligned with Cartesian directions
    • Prismatic X, Y, Z
    • Spherical (with and withoug singularities)
    • FreeFlyer (i.e. no constraint, for mobile robots like humanoids -- using quaternion representation for the rotation)
        • Fixed (concatenation of two consecutive bodies)


[Unreleased]: https://github.com/stack-of-tasks/pinocchio/compare/v2.7.1...HEAD
[2.7.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.7.0...v2.7.1
[2.7.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.21...v2.7.0
[2.6.21]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.20...v2.6.21
[2.6.20]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.19...v2.6.20
[2.6.19]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.18...v2.6.19
[2.6.18]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.17...v2.6.18
[2.6.17]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.16...v2.6.17
[2.6.16]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.15...v2.6.16
[2.6.15]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.14...v2.6.15
[2.6.14]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.13...v2.6.14
[2.6.13]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.12...v2.6.13
[2.6.12]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.11...v2.6.12
[2.6.11]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.10...v2.6.11
[2.6.10]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.9...v2.6.10
[2.6.9]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.8...v2.6.9
[2.6.8]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.7...v2.6.8
[2.6.7]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.6...v2.6.7
[2.6.6]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.5...v2.6.6
[2.6.5]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.4...v2.6.5
[2.6.4]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.3...v2.6.4
[2.6.3]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.2...v2.6.3
[2.6.2]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.1...v2.6.2
[2.6.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.6.0...v2.6.1
[2.6.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.6...v2.6.0
[2.5.6]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.5...v2.5.6
[2.5.5]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.4...v2.5.5
[2.5.4]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.3...v2.5.4
[2.5.3]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.2...v2.5.3
[2.5.2]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.1...v2.5.2
[2.5.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.5.0...v2.5.1
[2.5.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.7...v2.5.0
[2.4.7]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.6...v2.4.7
[2.4.6]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.5...v2.4.6
[2.4.5]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.4...v2.4.5
[2.4.4]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.3...v2.4.4
[2.4.3]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.2...v2.4.3
[2.4.2]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.1...v2.4.2
[2.4.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.4.0...v2.4.1
[2.4.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.3.1...v2.4.0
[2.3.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.3.0...v2.3.1
[2.3.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.2.3...v2.3.0
[2.2.3]: https://github.com/stack-of-tasks/pinocchio/compare/v2.2.2...v2.2.3
[2.2.2]: https://github.com/stack-of-tasks/pinocchio/compare/v2.2.1...v2.2.2
[2.2.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.2.0...v2.2.1
[2.2.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.11...v2.2.0
[2.1.11]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.10...v2.1.11
[2.1.10]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.9...v2.1.10
[2.1.9]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.8...v2.1.9
[2.1.8]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.7...v2.1.8
[2.1.7]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.6...v2.1.7
[2.1.6]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.5...v2.1.6
[2.1.5]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.4...v2.1.5
[2.1.4]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.3...v2.1.4
[2.1.3]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.2...v2.1.3
[2.1.2]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.1...v2.1.2
[2.1.1]: https://github.com/stack-of-tasks/pinocchio/compare/v2.1.0...v2.1.1
[2.1.0]: https://github.com/stack-of-tasks/pinocchio/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/stack-of-tasks/pinocchio/compare/v1.3.3...v2.0.0
[1.3.3]: https://github.com/stack-of-tasks/pinocchio/compare/v1.3.2...v1.3.3
[1.3.2]: https://github.com/stack-of-tasks/pinocchio/compare/v1.3.1...v1.3.2
[1.3.1]: https://github.com/stack-of-tasks/pinocchio/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.9...v1.3.0
[1.2.9]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.8...v1.2.9
[1.2.8]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.7...v1.2.8
[1.2.7]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.6...v1.2.7
[1.2.6]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.5...v1.2.6
[1.2.5]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.4...v1.2.5
[1.2.4]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.3...v1.2.4
[1.2.3]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.1...v1.2.3
[1.2.1]: https://github.com/stack-of-tasks/pinocchio/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/stack-of-tasks/pinocchio/compare/v1.1.2...v1.2.0
[1.1.2]: https://github.com/stack-of-tasks/pinocchio/compare/v1.1.0...v1.1.2
[1.1.0]: https://github.com/stack-of-tasks/pinocchio/compare/v1.0.2...v1.1.0
[1.0.2]: https://github.com/stack-of-tasks/pinocchio/compare/v1.0.0...v1.0.2
[1.0.0]: https://github.com/stack-of-tasks/pinocchio/releases/tag/v1.0.0
