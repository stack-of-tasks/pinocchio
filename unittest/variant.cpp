#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

int main()
{
  using namespace Eigen;
  using namespace se3;


  JointModelVariant jmodel = JointModelRX();
  const JointDataVariant & jdata = CreateJointData::run(jmodel);

  JointDataGeneric jdatagen(jdata);
  JointModelGeneric jmodelgen(jmodel);

  se3::Model model;
}
