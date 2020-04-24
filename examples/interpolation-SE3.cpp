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

  // Starting configuration
  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = 1 ; pose_s(3) = -0.13795 ; 
  pose_s(4) = 0.13795; pose_s(5) = 0.69352; pose_s(6) = 0.69352;
  aSE3.normalize(pose_s);

  // Goal configuration
  pose_g(0) = 4; pose_g(1) = 3;
  pose_g(2) = 3 ; pose_g(3) = -0.46194;
  pose_g(4) = 0.331414; pose_g(5) = 0.800103; pose_g(6) = 0.191342; 
  aSE3.normalize(pose_g);

  SE3Operation::ConfigVector_t pole_u;
  aSE3.interpolate(pose_s,pose_g,0.5, pole_u);
  std::cout << "Interpolated configuration: " << pole_u.transpose() << std::endl;

  return 0;
}

