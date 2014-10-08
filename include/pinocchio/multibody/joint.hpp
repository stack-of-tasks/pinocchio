#ifndef __se3_joint_hpp__
#define __se3_joint_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"

namespace se3
{
  enum { MAX_JOINT_NV = 6 }; 
}

#endif // ifndef __se3_joint_hpp__
