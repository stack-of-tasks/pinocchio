//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{

  template<typename D>
  void addJointAndBody(Model & model,
                       const JointModelBase<D> & jmodel,
                       const Model::JointIndex parent_id,
                       const SE3 & joint_placement,
                       const std::string & name,
                       const Inertia & Y)
  {
    Model::JointIndex idx;
    typedef typename D::TangentVector_t TV;
    typedef typename D::ConfigVector_t CV;
    
    idx = model.addJoint(parent_id,jmodel,joint_placement,
                         name + "_joint",
                         TV::Zero(),
                         1e3 * (TV::Random() + TV::Constant(1)),
                         1e3 * (CV::Random() - CV::Constant(1)),
                         1e3 * (CV::Random() + CV::Constant(1))
                         );
    
    model.appendBodyToJoint(idx,Y,SE3::Identity());
  }

  void buildAllJointsModel(Model & model)
  {
    addJointAndBody(model,JointModelFreeFlyer(),model.getJointId("universe"),SE3::Identity(),"freeflyer",Inertia::Random());
    addJointAndBody(model,JointModelSpherical(),model.getJointId("freeflyer_joint"),SE3::Identity(),"spherical",Inertia::Random());
    addJointAndBody(model,JointModelPlanar(),model.getJointId("spherical_joint"),SE3::Identity(),"planar",Inertia::Random());
    addJointAndBody(model,JointModelRX(),model.getJointId("planar_joint"),SE3::Identity(),"rx",Inertia::Random());
    addJointAndBody(model,JointModelPX(),model.getJointId("rx_joint"),SE3::Identity(),"px",Inertia::Random());
    addJointAndBody(model,JointModelPrismaticUnaligned(SE3::Vector3(1,0,0)),model.getJointId("px_joint"),SE3::Identity(),"pu",Inertia::Random());
    addJointAndBody(model,JointModelRevoluteUnaligned(SE3::Vector3(0,0,1)),model.getJointId("pu_joint"),SE3::Identity(),"ru",Inertia::Random());
    addJointAndBody(model,JointModelSphericalZYX(),model.getJointId("ru_joint"),SE3::Identity(),"sphericalZYX",Inertia::Random());
    addJointAndBody(model,JointModelTranslation(),model.getJointId("sphericalZYX_joint"),SE3::Identity(),"translation",Inertia::Random());
  }

}
