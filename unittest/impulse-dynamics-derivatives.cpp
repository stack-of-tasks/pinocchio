//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/impulse-dynamics.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_derivatives_no_contact)
{
  //result: (dMdq)(dqafter-v) = drnea(q,0,dqafter-v)
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  
  // Contact models and data
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) empty_contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) empty_contact_data;

  const double mu0 = 0.;
  const double r_coeff = 0.;
  
  initContactDynamics(model,data,empty_contact_models);
  impulseDynamics(model,data,q,v,empty_contact_models,empty_contact_data,r_coeff, mu0);

  const Eigen::VectorXd dv = data.dq_after-v;
  computeImpulseDynamicsDerivatives(model,data,empty_contact_models,empty_contact_data,mu0);

  Motion gravity_bk = model.gravity;
  model.gravity.setZero();
  computeRNEADerivatives(model, data_ref, q, Eigen::VectorXd::Zero(model.nv), dv);
  // Reference values
  BOOST_CHECK(data_ref.dtau_dq.isApprox(data.dtau_dq));
}

BOOST_AUTO_TEST_CASE(test_sparse_impulse_dynamics_derivatives)
{
  using namespace Eigen;
  using namespace pinocchio;
  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  //  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) contact_data;

  RigidContactModel ci_LF(CONTACT_6D,model.getFrameId(LF),LOCAL);
  RigidContactModel ci_RF(CONTACT_3D,model.getFrameId(RF),LOCAL);

  contact_models.push_back(ci_LF); contact_data.push_back(RigidContactData(ci_LF));
  contact_models.push_back(ci_RF); contact_data.push_back(RigidContactData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  const double r_coeff = 0.5;
  initContactDynamics(model,data,contact_models);
  impulseDynamics(model,data,q,v,contact_models,contact_data,r_coeff,mu0);
  computeImpulseDynamicsDerivatives(model,data,contact_models,contact_data,mu0);
  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceVector;
  
  ForceVector iext((size_t)model.njoints);
  for(ForceVector::iterator it = iext.begin(); it != iext.end(); ++it)
    (*it).setZero();
  iext[model.getJointId(LF)] = model.frames[model.getFrameId(LF)].placement.act(contact_data[0].contact_force);
  iext[model.getJointId(RF)] = model.frames[model.getFrameId(RF)].placement.act(contact_data[1].contact_force);
  
  Motion gravity_bk = model.gravity;
  model.gravity.setZero();
  computeRNEADerivatives(model, data_ref, q, Eigen::VectorXd::Zero(model.nv),
                         data.ddq, iext);  
  model.gravity = gravity_bk;

  BOOST_CHECK(data_ref.dtau_dq.isApprox(data.dtau_dq));  
}



BOOST_AUTO_TEST_SUITE_END ()
