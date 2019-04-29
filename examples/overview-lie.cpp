/// (c) Copyright CNRS LAAS 2019
/// Olivier Stasse
#include <iostream>
#include "pinocchio/multibody/liegroup/liegroup.hpp"

using namespace pinocchio;
int main()
{
  typedef double Scalar;
  enum {Options = 0};
  SpecialEuclideanOperationTpl<2,Scalar,Options> aSE2;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::ConfigVector_t pose_s,pose_g;
  SpecialEuclideanOperationTpl<2,Scalar,Options>::TangentVector_t delta_u;

  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = cos(M_PI/4.0); pose_s(3) = sin(M_PI/4.0);
  pose_g(0) = 3.0; pose_g(1) = -1.0;
  pose_g(2) = cos(-M_PI/2.0); pose_g(3) = sin(-M_PI/2.0);

  aSE2.difference(pose_s,pose_g,delta_u);
  std::cout << delta_u << std::endl;

  aSE2.difference(pose_s,pose_g,delta_u);

  SpecialEuclideanOperationTpl<2,Scalar,Options>::ConfigVector_t pose_check;
  aSE2.integrate(pose_s,delta_u,pose_check);
  std::cout << pose_check << std::endl;
}
    
