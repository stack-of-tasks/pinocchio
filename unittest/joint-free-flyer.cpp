//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(Model & model,
                     const JointModelBase<D> & jmodel,
                     const Model::JointIndex parent_id,
                     const SE3 & joint_placement,
                     const std::string & joint_name,
                     const Inertia & Y)
{
  Model::JointIndex idx;
  
  idx = model.addJoint(parent_id,jmodel,joint_placement,joint_name);
  model.appendBodyToJoint(idx,Y);
}
