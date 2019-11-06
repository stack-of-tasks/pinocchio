#include <iostream>
#include "pinocchio/multibody/liegroup/liegroup.hpp"

using namespace pinocchio;

int main()
{
  typedef double Scalar;
  enum {Options = 0};

  typedef SpecialEuclideanOperationTpl<2,Scalar,Options> SE2Operation;
  SE2Operation aSE2;
  SE2Operation::ConfigVector_t pose_s,pose_g;
  SE2Operation::TangentVector_t delta_u;

  // Starting configuration
  pose_s(0) = 1.0; pose_s(1) = 1.0;
  pose_s(2) = cos(M_PI/4.0); pose_s(3) = sin(M_PI/4.0);

  // Goal configuration
  pose_g(0) = 3.0; pose_g(1) = -1.0;
  pose_g(2) = cos(-M_PI/2.0); pose_g(3) = sin(-M_PI/2.0);

  // Computes the differences (expressed in the tangent space of the configuration space) between 
  // the starting and the goal configuration
  aSE2.difference(pose_s,pose_g,delta_u);
  std::cout << "difference: " << delta_u.transpose() << std::endl;

  // Check that the composition of the starting configuration and the difference vector gives the goal configuration
  SE2Operation::ConfigVector_t pose_check;
  aSE2.integrate(pose_s,delta_u,pose_check);
  std::cout << "goal configuration (from composition): " << pose_check.transpose() << std::endl;
  std::cout << "goal configuration: " << pose_g.transpose() << std::endl;
}
    
