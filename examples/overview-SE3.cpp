#include <iostream>
#include "pinocchio/multibody/liegroup/liegroup.hpp"

using namespace pinocchio;
int main ()
{ 
  typedef double Scalar;
  typedef SpecialEuclideanOperationTpl<3,Scalar> SE3Operation;

  SE3Operation aSE3;
  SE3Operation::ConfigVector_t pose_s,pose_g;
  SE3Operation::TangentVector_t delta_u ;  

  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = 1 ; pose_s(3) = -0.13795 ; 
  pose_s(4) = 0.13795; pose_s(5) = 0.69352; pose_s(6) = 0.69352;
  pose_g(0) = 4; pose_g(1) = 3 ;
  pose_g(2) = 3 ; pose_g(3) = -0.46194;
  pose_g(4) = 0.331414; pose_g(5) = 0.800103; pose_g(6) = 0.191342; 

  // First normalize the inputs
  aSE3.normalize(pose_s);
  std::cout << "pose_s: " << pose_s.transpose() << std::endl;
  aSE3.normalize(pose_g);
  std::cout << "pose_g: " << pose_g.transpose() << std::endl;

  aSE3.difference(pose_s,pose_g,delta_u);
  std::cout << "delta_u: " << delta_u.transpose() << std::endl;

  SE3Operation::ConfigVector_t pose_check;

  aSE3.integrate(pose_s,delta_u,pose_check);
  std::cout << "pose_check: " << pose_check.transpose() << std::endl;

  return 0;
}

