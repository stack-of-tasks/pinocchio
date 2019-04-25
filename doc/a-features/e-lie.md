
# Dealing with Lie group geometry 

Pinocchio is relying heavily on Lie groups and Lie algebra to handle motions and more specifically rotations.
For this reason it supports the following special groups \\( SO(2), SO(3), SE(2), SE(3) \\) and implements their associated algebra 
\f$( \mathfrak{se}(2) , \mathfrak{se}(3) \f$).
It has various applications like representing the motion of a robot free flyer joint (typically the base of a mobile robot),
or the motion of the robot links. The later is particularly useful for collision detection.
It is also interesting to have general vector space over which a Lie algebra is defined. 

## Using \\( SE(2) \\) with pinocchio in C++

As a motivating example let us consider a mobile robot evolving in a plane \f$(\mathbb{R}^2 \times \mathbb{S}^1 \f$).
![SE2MotivatingExample.svg](SE2MotivatingExample.svg)

The robot starts at position \f$ pose_s=(x_s,y_s,\theta_s) \f$ and after a rigid motion
\f$ \delta_u=(\delta x,\delta y,\delta \theta) \f$ 
it is finishing 
at position \f$ pose_g = (x_g,y_g,\theta_g)\f$.
It is possible to instantiate the corresponding \\(SE(2)\\) objects using:

\code
  typedef double Scalar;
  enum {Options = 0};
  
  SpecialEuclideanOperationTpl<2,Scalar,Options> aSE2;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::ConfigVector_t pose_s,pose_g;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::TangentVector_t delta_u;
\endcode
You can change Scalar by another type such as float.


In this example, \f$pose_s=(1,1,\pi/4)\f$ and \f$ pose_g=(3,1,-\pi/2) \f$ and we want to compute 
\f$ \delta_u \f$
\code
  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = cos(M_PI/4.0); pose_s(3) = sin(M_PI/4.0);
  pose_g(0) = 3.0; pose_g(1) = -1.0;
  pose_g(2) = cos(-M_PI/2.0); pose_g(3) = sin(-M_PI/2.0);

  aSE2.difference(pose_s,pose_g,delta_u);
  std::cout << delta_u << std::endl;
\endcode

aSE2 is used to compute the difference between two configuration vectors representing the two poses. Note that the rotation is represented by two numbers \f$(sin(\theta),cos(\theta))\f$ which is also a \f$ SO(2) \f$ object.
The difference lies in the tangent space of \f$SE(3)\f$ and is representend by a vector of 3 reals.
Therefore the output is:
\code
 3.33216
-1.38023
-2.35619
\endcode
Note that the linear part is not following a straight path, it also takes into account
that the system is rotating.

We can verify that this is the appropriate motion by integrating:
\code
  SpecialEuclideanOperationTpl<2,Scalar,Options>::ConfigVector_t pose_check;
  aSE2.integrate(pose_s,delta_u,pose_check);
  std::cout << pose_check << std::endl;
\endcode

The result is indeed:

\code
3
-1
 0
-1
\endcode



