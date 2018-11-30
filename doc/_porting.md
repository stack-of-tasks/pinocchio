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


