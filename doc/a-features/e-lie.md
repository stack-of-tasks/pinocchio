
# Dealing with Lie group geometry 

Pinocchio relies heavily on Lie groups and Lie algebras to handle motions and more specifically rotations.
For this reason it supports the following special groups \\( SO(2), SO(3), SE(2), SE(3) \\) and implements their associated algebras
\f$ \mathfrak{se}(2) , \mathfrak{se}(3) \f$.
It has various applications like representing the motion of a robot free flyer joint (typically the base of a mobile robot),
or the motion of the robot links. The later is particularly useful for collision detection.
It is also interesting to have general vector space over which a Lie algebra is defined. 


## Using \\( SE(2) \\) with pinocchio in C++

As a motivating example let us consider a mobile robot evolving in a plane \f$(\mathbb{R}^2 \times \mathbb{S}^1 \f$).
![SE2MotivatingExample](SE2MotivatingExample.svg)

The robot starts at position \f$ pose_s = (x_s,y_s,\theta_s) \f$ and after a rigid motion
\f$ \delta_u=(\delta x,\delta y,\delta \theta) \f$ 
it finishes 
at \f$ pose_g = (x_{g},y_{g},\theta_{g})\f$.
It is possible to instantiate the corresponding \\(SE(2)\\) objects using:

\code
  typedef double Scalar;
  enum {Options = 0};
  
  SpecialEuclideanOperationTpl<2,Scalar,Options> aSE2;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::ConfigVector_t pose_s,pose_g;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::TangentVector_t delta_u;
\endcode
You can change Scalar by another type such as float.


In this example, \f$ pose_s=(1,1,\pi/4)\f$ and \f$ pose_g=(3,1,-\pi/2) \f$ and we want to compute 
\f$ \delta_u \f$
\code
  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = cos(M_PI/4.0); pose_s(3) = sin(M_PI/4.0);
  pose_g(0) = 3.0; pose_g(1) = -1.0;
  pose_g(2) = cos(-M_PI/2.0); pose_g(3) = sin(-M_PI/2.0);

  aSE2.difference(pose_s,pose_g,delta_u);
  std::cout << delta_u << std::endl;
\endcode

aSE2 is used to compute the difference between two configuration vectors representing the two poses. Note that the rotation is represented by two numbers \f$ sin(\theta),cos(\theta)\f$ which is also a \f$ SO(2) \f$ object.
The difference lies in the tangent space of \f$ SE(2)\f$ and is representend by a vector of 3 reals.
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

## Using \f$ SE(3) \f$ with pinocchio in C++

Our mobile robot is not in a plane but in a 3-dimensional space. So let's consider a object in our physical space. This is actually almost the same case, we want the object from one position to an other position. The difficulty lies in the fact that we now have three dimensions so the object has six degrees of freedom, three corresponding to its translation and three to its rotation. 

![SE3MotivatingExample](SE3Example1.jpg)

It is also possible to instantiate the corresponding object which is now a \f$ SE(3) \f$ object using the same algorithm and changing the dimension parameter:

\code
  typedef double Scalar;
  enum {Options = 0};
  
  SpecialEuclideanOperationTpl<3,Scalar,Options> aSE3 ;
  SpecialEuclideanOperationTpl<3,Scalar,Options>::ConfigVector_t pose_s,pose_g;
  SpecialEuclideanOperationTpl<3,Scalar,Options>::TangentVector_t delta_u ;
\endcode

In this example, \f$ pose_s=(1,1,1,\pi/2,\pi/4,\pi/8)\f$ and \f$ pose_g=(4,3,3,\pi/4,\pi/3, -\pi) \f$. For the starting position, there is first a rotation around the y-axis then the x-axis and finally the z-axis. For the final position, the rotations are in this order, x-axis, y-axis, z-axis. We want to compute \f$ \delta_u \f$.

- For the first pose, we have the three rotations matrices for each rotation :

  \f$ R_{x_s} = 
  \begin{bmatrix} 1 &0 &0 \\ 0 &cos(\pi/8) &-sin(\pi/8) \\0 &sin(\pi/8) &cos(\pi/8) \end{bmatrix}  \ \  R_{y_s}  =  \begin{bmatrix} cos(\pi/4) &0 &sin(\pi/4) \\ 0 &1 &0 \\-sin(\pi/4) &0 &cos(\pi/4) \end{bmatrix} \ \ R_{z_s}  =  \begin{bmatrix} cos(\pi/2) &-sin(\pi/2) &0 \\sin(\pi/2) &cos(\pi/2) &0 \\ 0 &0 &1\end{bmatrix} \f$

  Therefore, the complete rotation is:

  \f$ R_{pose_s} = R_{s_z} * R_{s_x} * R_{s_y}  = \begin{bmatrix} 0 &-1 &0 \\ cos(\pi/4)*cos(\pi/8) + sin(\pi/4) * sin(\pi/8) &0 &sin(\pi/4) * cos(\pi/8) - cos(\pi/4) * sin(\pi/8) \\ sin(\pi/8) * cos(\pi/4) - cos(\pi/8) * sin(\pi/4) &0 &sin(\pi/4) * sin(\pi/8) + cos(\pi/4) * cos(\pi/8) \end{bmatrix} \f$

- For the second one, we have:

  \f$ R_{x_g} = 
  \begin{bmatrix} 1 &0 &0 \\ 0 &cos(\pi/4) &-sin(\pi/4) \\0 &sin(\pi/4) &cos(\pi/4) \end{bmatrix}  \ \  R_{y_g}  =  \begin{bmatrix} cos(\pi/3) &0 &sin(\pi/3) \\ 0 &1 &0 \\ -sin(\pi/3) &0 &cos(\pi/3) \end{bmatrix} \ \ R_{z_g} =  \begin{bmatrix} cos(-\pi) &-sin(-\pi) &0 \\ sin(-\pi) &cos(-\pi) &0 \\ 0 &0 &1\end{bmatrix} \f$

  The complete rotation is:

  \f$ R_{pose_g} =
  \begin{bmatrix} -cos(\pi/3) &-sin(\pi/3) * sin(\pi/4) &-cos(\pi/4) * sin(\pi/3) \\ 0 &-cos(\pi/4) &sin(\pi/4) \\ -sin(\pi/3) &sin(\pi/4) * cos(\pi/3) &cos(\pi/3) * cos(\pi/4) \end{bmatrix} \f$


To compute \f$ \delta_u \f$ using Pinocchio we need to transform \f$ R_{pose_s} \f$ and \f$ R_{pose_g} \f$ matrices into quaternions using:

\code
  float s = 0.5f / sqrtf(trace+ 1.0f);
  q.x = ( R[2][1] - R[1][2] ) * s;
  q.y = ( R[0][2] - R[2][0] ) * s;
  q.z = ( R[1][0] - R[0][1] ) * s;
  q.w = 0.25f / s;
\endcode

The quaternions components are:
- For the first rotation
  \code
  0.69352
  -0.13795
  0.13795
  0.69352
  \endcode

- For the second one
  \code
    0.191342
    -0.46194
    0.331414
    0.800103
  \endcode

For each pose we have now a mathematical object with seven components and both are normalized. As for the \f$ SE(2) \f$ example we compute \f$ \delta_u \f$ using:

\code
  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = 1 ; pose_s(3) = -0.13795 ; 
  pose_s(4) = 0.13795; pose_s(5) = 0.69352; pose_s(6) = 0.69352;
  pose_g(0) = 4; pose_g(1) = 3 ;
  pose_g(2) = 3 ; pose_g(3) = -0.46194;
  pose_g(4) = 0.331414; pose_g(5) = 0.800103; pose_g(6) = 0.191342; 

  aSE3.difference(pose_s,pose_g,delta_u);
  std::cout << delta_u << std::endl;
\endcode

The difference lies in the tangent space of \f$ SE(3)\f$ and is represented by a vector of 6 reals which is:

\code 
  -1.50984
  -3.58755
  2.09496
  -0.374715
  0.887794
  0.86792
\endcode

The three first values are linear and the three last are velocities.

To verify it is the good solution, we integrate:

\code
  SpecialEuclideanOperationTpl<3,Scalar,Options>::ConfigVector_t pose_check;

  aSE3.integrate(pose_s,delta_u,pose_check);
  std::cout << pose_check << std::endl;
\endcode

Indeed, we find :

\code
  4
  3
  3
  -0.46194
  0.331414
  0.800103
  0.191234
\endcode 



## Using interpolation to plot a trajectory with pinocchio in C++

Assuming that we want to make the robot pass through known positions, we can use interpolations to plot a trajectory.

The problem is an interpolation such as Lagrange's one only takes into account translations whlie the robot interact with its environment by performing translations and rotations.

A possibility is to use the \f$ \delta_{theta} \f$ method by using quaternions. The method is simple, we just vary the angle, the scalar component of the quaternion, with very small variations.

Let's consider the previous example, we can interpolate trajectory using:
\code
  SpecialEuclideanOperationTpl<3,Scalar,Options>::ConfigVector_t pole;
  aSE3.interpolate(pose_s,pose_g,0.5f, pole);
  std::cout << pole << std::endl;
\endcode

The output corresponds to the middle of the trajectory and is:
\code
  2.7486
  1.4025
  2.22461
  -0.316431
  0.247581
  0.787859
  0.466748
\endcode



