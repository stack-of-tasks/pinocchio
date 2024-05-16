# Porting from Pinocchio 2 to 3

\section PortingIntro3 Contents

This section describes how to port your code from the Pinocchio 2 to Pinocchio 3.

\section PortingC3 C++ changes

All Pinocchio 2 deprecated functions had been removed.

The following functions and headers have been renamed or moved:

- Replace `pinocchio::BiasZeroTpl` by `pinocchio::MotionZeroTpl`
- Replace `pinocchio::fusion::JointVisitorBase` by `pinocchio::fusion::JointUnaryVisitorBase`
- Replace `pinocchio::fusion::push_front` by `pinocchio::fusion::append`
- Replace `pinocchio::regressor::computeStaticRegressor` by `pinocchio::computeStaticRegressor`
- Replace `pinocchio::jointJacobian` by `pinocchio::computeJointJacobian`
- Replace `pinocchio::frameJacobian` by `pinocchio::computeFrameJacobian`
- Replace `pinocchio::framesForwardKinematics` by `pinocchio::updateFramePlacements`
- Replace `pinocchio::kineticEnergy` by `pinocchio::computeKineticEnergy`
- Replace `pinocchio::potentialEnergy` by `pinocchio::computePotentialEnergy`
- Replace `pinocchio::computeCentroidalDynamics` by `pinocchio::computeCentroidalMomentum` and `pinocchio::computeCentroidalMomentumTimeVariation`
- Replace `pinocchio::centerOfMass(const ModelTpl&, DataTpl&, int, bool)` by `pinocchio::centerOfMass(const ModelTpl&, DataTpl&, KinematicLevel, bool)`
- Replace `pinocchio::copy(const ModelTpl&, const DataTpl&, DataTpl&, int)` by `pinocchio::copy(const ModelTpl&, const DataTpl&, DataTpl&, KinematicLevel)`
- Replace `pinocchio/algorithm/dynamics.hpp` by `pinocchio/algorithm/constrained-dynamics.hpp`
- Change the order of arguments in some of `pinocchio::GeometryObject`'s constructors

If the compiler complains about missing `pinocchio::computeDistances` or `pinocchio::computeCollisions` function you can either:
- Include the right headers:
  - `pinocchio/collision/collision.hpp`
  - `pinocchio/collision/distance.hpp`
- Define `PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2` when building your project

The following functions have been removed:

- Remove `pinocchio::setGeometryMeshScales`
- Remove some `pinocchio::forwardDynamics` signatures
- Remove some `pinocchio::impulseDynamics` signatures

The following functions and headers are now deprecated:

- Deprecate `pinocchio/algorithm/parallel/geometry.hpp` moved at `pinocchio/collision/parallel/geometry.hpp`
- Deprecate `pinocchio/spatial/fcl-pinocchio-conversions.hpp` moved at `pinocchio/collision/fcl-pinocchio-conversions.hpp`
- Deprecate `pinocchio/parsers/sample-models.hpp` moved at `pinocchio/multibody/sample-models.hpp`
- Deprecate `pinocchio/math/cppad.hpp` moved at `pinocchio/autodiff/cppad.hpp`
- Deprecate `pinocchio/math/cppadcg.hpp` moved at `pinocchio/autodiff/cppadcg.hpp`
- Deprecate `pinocchio/math/casadi.hpp` moved at `pinocchio/autodiff/casadi.hpp`
- Deprecate `pinocchio::FrameTpl::parent` replaced by `pinocchio::FrameTpl::parentJoint`
- Deprecate `pinocchio::FrameTpl::previousFrame` replaced by `pinocchio::FrameTpl::parentFrame`
- Deprecate `pinocchio/algorithm/contact-dynamics.hpp` algorithms replaced by `pinocchio/algorithm/constrained-dynamics.hpp`

\subsection PortingCTarget3 CMake targets

Pinocchio 3 has been split into multiple CMake targets:

- `pinocchio`: Link against all available Pinocchio libraries
- `pinocchio_headers`: Link against Pinocchio header only core library
- `pinocchio_double`: Link against Pinocchio core library explicitly template instantiated for double scalar type
- `pinocchio_casadi`: Link against Pinocchio core library explicitly template instantiated for casadi scalar type
- `pinocchio_cppad`: Link against Pinocchio core library explicitly template instantiated for cppad scalar type
- `pinocchio_cppadcg`: Link against Pinocchio core library explicitly template instantiated for cppadcg scalar type
- `pinocchio_parsers`: Link against Pinocchio parsers library
- `pinocchio_parallel`: Link against Pinocchio parallel algorithms library
- `pinocchio_collision`: Link against Pinocchio collision library
- `pinocchio_collision_parallel`: Link against Pinocchio collision parallel algorithms library
- `pinocchio_extra`: Link against Pinocchio extra algorithms library

\section PortingPy3 Python changes

All Pinocchio 2 deprecated functions had been removed.

The following functions have been renamed or moved:

- Replace `pinocchio.utils.skew` by `pinocchio.skew`
- Replace `pinocchio.utils.se3ToXYZQUAT` by `pinocchio.SE3ToXYZQUATtuple`
- Replace `pinocchio.utils.XYZQUATToSe3` by `pinocchio.XYZQUATToSE3`
- Replace `pinocchio.robot_wrapper.RobotWrapper.frameClassicAcceleration` by `pinocchio.robot_wrapper.RobotWrapper.frameClassicalAcceleration`
- Replace `pinocchio.robot_wrapper.RobotWrapper.jointJacobian` by `pinocchio.robot_wrapper.RobotWrapper.computeJointJacobian`
- Replace `pinocchio.robot_wrapper.RobotWrapper.frameJacobian` by `pinocchio.robot_wrapper.RobotWrapper.computeFrameJacobian`
- Replace `pinocchio.robot_wrapper.RobotWrapper.initDisplay` by `pinocchio.robot_wrapper.RobotWrapper.initViewer`
- Replace `pinocchio.robot_wrapper.RobotWrapper.loadDisplayModel` by `pinocchio.robot_wrapper.RobotWrapper.loadViewerModel`
- Replace `pinocchio.deprecated.se3ToXYZQUATtuple` by `pinocchio.SE3ToXYZQUATtuple`
- Replace `pinocchio.deprecated.se3ToXYZQUAT` by `pinocchio.SE3ToXYZQUAT`
- Replace `pinocchio.deprecated.XYZQUATToSe3` by `pinocchio.XYZQUATToSE3`
- Replace `pinocchio.deprecated.buildGeomFromUrdf(model, filename, [str])` by `pinocchio.buildGeomFromUrdf(model, filename, type, package_dirs, mesh_loader)`
- Replace `pinocchio.rpy.npToTTuple` by `pinocchio.utils.npToTTuple`
- Replace `pinocchio.rpy.npToTuple` by `pinocchio.utils.npToTuple`
- Replace `pinocchio.jacobianSubtreeCoMJacobian` by `pinocchio.jacobianSubtreeCenterOfMass`

The following functions have been removed:

- Remove `pinocchio.utils.cross`
- Remove `pinocchio.robot_wrapper.RobotWrapper.initMeshcatDisplay`
- Remove `pinocchio.deprecated.setGeometryMeshScales` by `pinocchio`


# Porting from Pinocchio 1.3.3 to 2.0.0

\section PortingIntro What is included

This section describes how to port your code from the latest Pinocchio 1 release (1.3.3) to 2.0.

Note that this section does not cover API changes that were made *before* Pinocchio 1.3.3.
Therefore, if you are still using an older version of Pinocchio 1,
it is recommended that before you switch to 2.0 you should upgrade to 1.3.3 and make sure everything still works.
In particular, remove all calls to deprecated methods and replace them appropriately.

The vast majority of the changes took place in C++.

\section PortingC Changes in C++
Although the class system was heavily re-worked, it should not make a lot of difference from the user's point of view.
Relevant changes are listed below.

\subsection PortingCHeaderonly Header-only
Pinocchio is now fully header-only. This means you do not have to link to the Pinocchio library when compiling your code.
On the other hand, you might need to link to additional system libraries.

\subsection PortingCNamespace Namespace
The most important change is the namespace.
Now, the top-level Pinocchio namespace is not `se3` anymore, but `pinocchio`.

Therefore, all occurrences of `se3` in your code should be replaced by `pinocchio`.

If you feel like "pinocchio" is too long to type but you do not want to employ `using namespace pinocchio`,
you can use
```
namespace pin = pinocchio;
```

Your code will not compile if you try to use namespace `se3`.
In order to make it work, you need to compile it with the following flag
```
-DPINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1
```

\subsection PortingCMacros Deprecated macros

The following marcos are not employed anymore
```
WITH_HPP_FCL
WITH_URDFDOM
WITH_LUA5
```
in favor of
```
PINOCCHIO_WITH_HPP_FCL
PINOCCHIO_WITH_URDFDOM
PINOCCHIO_WITH_LUA5
```
Therefore, you now need to issue the new macros in your compilation commands.

If you are using them in your code, in order to make them work, you can do
```
-DPINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1
```

\subsection PortingCSignature Method signatures
Many methods which were taking a `ReferenceFrame = {WORLD/LOCAL}` enum as template parameter, such as `getJointJacobian`,
are now deprecated. The reference frame is now passed as an input.
For instance, you should switch from
```
getJointJacobian<LOCAL>(model,data,jointId,J);
```
to
```
getJointJacobian(model,data,jointId,LOCAL,J);
```

Notice that in principle your old code will still work, but you will get deprecation warnings.

\section PortingPython Changes in Python
Changes in Python are relatively minor.

\subsection PortingPythonNamespace Namespace

No real changes took place, but you are encouraged to stop using the idiom `import pinocchio as se3`.

From now on, the recommended practice is `import pinocchio as pin`.

\subsection PortingPythonRobotWrapper RobotWrapper

The constructor signature has changed from
```
__init__(self, filename, package_dirs=None, root_joint=None, verbose=False)
```
to
```
__init__(self, model = pin.Model(), collision_model = None, visual_model = None, verbose=False)
```
so that a model can be directly provided without the need of resorting to URDF.
To recover the previous construction technique, you can use the static method
```
RobotWrapper.BuildFromURDF(filename, package_dirs=None, root_joint=None, verbose=False)
```

Also, the signature
```
jointJacobian(self, q, index, update_kinematics=True, local_frame=True)
```
was changed to
```
jointJacobian(self, q, index, rf_frame=pin.ReferenceFrame.LOCAL, update_kinematics=True)
```
which should not change anything if you were only ever calling it with two arguments.
