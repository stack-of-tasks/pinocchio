#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"




int main()
{
  using namespace Eigen;
  using namespace se3;


  JointModelVariant jmodel = JointModelRX(0,0);
  const JointDataVariant & jdata = CreateJointData::run(jmodel);


  


  se3::Model model;
}
