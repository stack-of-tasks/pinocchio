# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added

- Add inverse dynamics (`rnea`) Python and C++ example ([#2083](https://github.com/stack-of-tasks/pinocchio/pull/2083))

### Fixed

- Re-initialize `Ycrb[0]` in `crbaMinimal` ([#2040](https://github.com/stack-of-tasks/pinocchio/pull/2040))
- Fix custom scalar use in `log` function ([#2047](https://github.com/stack-of-tasks/pinocchio/pull/2047))
- Raise exception on wrong input size in `XYZQUATToSE3` Python binding function ([#2073](https://github.com/stack-of-tasks/pinocchio/pull/2073))
- Remove memory leak in `buildGeomFromUrdf` and `buildGeomFromUrdfString` Python binding functions ([#2082]()https://github.com/stack-of-tasks/pinocchio/pull/2082)
- Fix Panda3D viewer examples ([#2087](https://github.com/stack-of-tasks/pinocchio/pull/2087))

### Changed

- Rename freeflyer_joint to root_joint in `humanoid` sample model ([#2043](https://github.com/stack-of-tasks/pinocchio/pull/2043))
- CMake minimal version is now 3.10 ([#2055](https://github.com/stack-of-tasks/pinocchio/pull/2055))
- Split headers and sources in different directories to have a more standard C++ project ([#2070](https://github.com/stack-of-tasks/pinocchio/pull/2070))

### Removed

- Remove support to `hpp-fcl` < v2.0.0 ([#2086](https://github.com/stack-of-tasks/pinocchio/pull/2086))
