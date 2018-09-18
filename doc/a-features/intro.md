# Main features of Pinocchio

Pinocchio has been written in C++ for efficiency reasons and uses the
Eigen library for linear algebra routines. It comes with
Python bindings for easy code prototyping. We describe here the main
features implemented in Pinocchio.

## Spatial algebra

Spatial algebra is a mathematical notation
commonly employed in rigid body dynamics to represent and manipulate
physical quantities such as velocities, accelerations and forces.
Pinocchio is based on this mathematical notation. Dedicated classes are
provided to represent coordinate transformations in the 3D Euclidean
space (named SE3), spatial motion vectors (Motion), spatial force
vectors (Force), and spatial inertias (Inertia). Along with the
available methods, this endows Pinocchio with an efficient software
library for spatial algebra calculations.

## Model and data

A fundamental paradigm of Pinocchio is the strict separation between
*model* and *data*. By *model*, we mean the physical description of the
robot, including kinematic and possibly inertial parameters defining its
structure. This information is held by a dedicated class which, once
created, is never modified by the algorithms of Pinocchio. By *data*, we
mean all values which are the result of a computation. *Data* vary
according to the joint configuration, velocity, etc\... of the system.
It contains for instance the velocity and the acceleration of each link.
It also stores intermediate computations and final results of the
algorithms in order to prevent memory allocation. With this splitting,
all the algorithms in Pinocchio follow the signature:

```
algorithm(model, data, arg1, arg2, ...)
```

where are the arguments of the function (e.g. configuration or
velocity). Keeping model and data separated reduces memory footprint
when performing several different tasks on the same robot, notably when
this involves parallel computation. Each process can employ its own data
object, while sharing the same model object. The fact that a model
object never changes within an algorithm of Pinocchio enhances the
predictability of the code.

A model can be created using the C++ API or loaded from an external
file, which can be either URDF, Lua (following the RBDL standard) or
Python.

## Supported kinematic models

Within a model, a robot is represented as a kinematic tree, containing a
collection of all the joints, information about their connectivity, and,
optionally, the inertial quantities associated to each link. In
Pinocchio a joint can have one or several degrees of freedom, and it
belongs to one of the following categories:
- **Revolute** joints, rotating around a fixed axis, either one of \f$X,Y,Z\f$ or a custom one;
- **Prismatic** joints, translating along any fixed axis, as in the revolute case;
- **Spherical** joints, free rotations in the 3D space;
- **Translation** joint, for free translations in the 3D space;
- **Planar** joints, for free movements in the 2D space;
- **Free-floating** joints, for free movements in the 3D space. Planar and free-floating joints are meant to be
  employed as the basis of kinematic tree of mobile robots (humanoids, automated vehicles, or objects in manipulation
  planning).
- More complex joints can be created as a collection of ordinary ones through the concept of **Composite** joint.

## Dealing with Lie group geometry

Each type of joints is characterized by its own specific configuration
and tangent spaces. For instance, the configuration and tangent spaces
of a revolute joint are both the real axis line \f$\mathbb{R}\f$, while for
a Spherical joint the configuration space corresponds to the set of
rotation matrices of dimension 3 and its tangent space is the space of
3-dimensional real vectors \f$\mathbb{R}^{3}\f$. Some configuration spaces
might not behave as a vector space, but have to be endowed with the
corresponding integration (exp) and differentiation (log) operators.
Pinocchio implements all these specific integration and differentiation
operators.

## Geometric models

Aside the kinematic model, Pinocchio defines a geometric model, i.e. the
volumes attached to the kinematic tree. This model can be used for
displaying the robot and computing quantities associated to collisions.
Like the kinematic model, the fixed quantities (placement and shape of
the volumes) are stored in a *GeometricModel* object, while buffers and
quantities used by associated algorithms are defined in a object. The
volumes are represented using the FCL library . Bodies
of the robot are attached to each joint, while obstacles of the
environment are defined in the world frame. Collision and distance
algorithms for the kinematic trees are implemented, based on FCL
methods.

## Main algorithms

The implementation of the basic algorithms, including all those listed
in this section, is recursive. The recursive formulation allows the
software to avoid repeated computations and to exploit the sparsity
induced by the kinematic tree. For the dynamics algorithms, we largely
drew inspiration from Featherstone algorithms, with slight
improvements.

### Forward kinematics

Pinocchio implements direct kinematic computations up to the second
order. When a robot configuration is given, a forward pass is performed
to compute the spatial placements of each joint and to store them as
coordinate transformations. If the velocity is given, it also computes
the spatial velocities of each joint (expressed in local frame), and
similarly for accelerations.

### Kinematic Jacobian

the spatial Jacobian of each joint can be easily computed with a single
forward pass, either expressed locally or in the world frame.

### Inverse dynamics

the Recursive Newton-Euler Algorithm (RNEA) computes the
inverse dynamics: given a desired robot configuration, velocity and
acceleration, the torques required to execute this motion are computed
and stored. The algorithm first performs a forward pass (equivalent to
second-order kinematics). It then performs a backward pass, computing
the wrenches transmitted along the structure and extracting the joint
torques needed to obtain the computed link motions. With the appropriate
inputs, this algorithm can also be employed to compute specific terms of
the dynamic model, such as the gravity effects.

### Joint space inertia matrix

the Composite Rigid Body Algorithm (CRBA) is
employed to compute the joint space inertia matrix of the robot. We have
implemented some slight modifications of the original algorithm that
improve the computational efficiency.

### Forward dynamics

the Articulated Body Algorithm (ABA)
computes the unconstrained forward dynamics: given a robot
configuration, velocity, torque and external forces, the resulting joint
accelerations are computed.

### Additional algorithms

beside the algorithms above, other methods are provided, most notably
for constrained forward dynamics, impulse dynamics, inverse of the joint
space inertia and centroidal
dynamics.

## Analytical derivatives

Beside proposing standard forward and inverse dynamics algorithms,
Pinocchio also provides efficient implementations of their analytical
derivatives. These derivatives are for
instance of primary importance in the context of whole-body trajectory
optimization or more largely, for numerical optimal control. To the best
of our knowledge, Pinocchio is the first rigid body framework which
implements this feature natively.

## Automatic differentiation and source code generation

In addition to analytical derivatives, Pinocchio supports automatic
differentiation. This is made possible through the full *scalar*
templatization of the whole C++ code and the use of any external library
that does automatic differentiation: ADOL-C, CasADi, CppAD and others. It is
important to keep in mind that these automatic derivatives are often
much slower than the analytical ones.

Another unique but central feature of Pinocchio is its ability to
generate code both at compile time and at runtime. This is achieved by
using another external toolbox called CppADCodeGen built on top of
CppAD. From any function using Pinocchio, CppADCodeGen is
able to generate on the fly its code in various languages: C, Latex,
etc. and to make some simplifications of the math expressions. Thanks to
this procedure, a code tailored for a specific robot model can be
generated and used externally to Pinocchio.
