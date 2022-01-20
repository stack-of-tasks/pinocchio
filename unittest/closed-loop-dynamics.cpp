//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/proximal.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace pinocchio;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(closed_loop_constraint_6D_LOCAL)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  
  RigidConstraintModel ci_RF_LF(CONTACT_6D,model,model.getJointId(RF),model.getJointId(LF),LOCAL);
  ci_RF_LF.joint1_placement.setRandom();
  ci_RF_LF.joint2_placement.setRandom();
  contact_models.push_back(ci_RF_LF);
  contact_datas.push_back(RigidConstraintData(ci_RF_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv);
  J_RF_local.setZero(); J_LF_local.setZero();
  getJointJacobian(model,data_ref,model.getJointId(RF),LOCAL,J_RF_local);
  getJointJacobian(model,data_ref,model.getJointId(LF),LOCAL,J_LF_local);
  
  const SE3 c1Mc2_ref = data_ref.oMi[ci_RF_LF.joint1_id].actInv(data_ref.oMi[ci_RF_LF.joint2_id]);
  
  J_ref = J_RF_local - c1Mc2_ref.toActionMatrix()*J_LF_local;
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = (data_ref.a[ci_RF_LF.joint1_id] - c1Mc2_ref.act(data_ref.a[ci_RF_LF.joint2_id])).toVector();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);

  BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());
  
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));
}

BOOST_AUTO_TEST_CASE(closed_loop_constraint_6D_LOCAL_WORLD_ALIGNED)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  
  const VectorXd q = randomConfiguration(model);
  const VectorXd v = VectorXd::Random(model.nv);
  const VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas;
  
  RigidConstraintModel ci_RF_LF(CONTACT_6D,model,model.getJointId(RF),model.getJointId(LF),LOCAL_WORLD_ALIGNED);
  ci_RF_LF.joint1_placement.setRandom();
  ci_RF_LF.joint2_placement.setRandom();
  contact_models.push_back(ci_RF_LF);
  contact_datas.push_back(RigidConstraintData(ci_RF_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  
  Eigen::MatrixXd J_ref(constraint_dim,model.nv);
  J_ref.setZero();
  
  computeAllTerms(model,data_ref,q,v);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  Data::Matrix6x J_RF_local(6,model.nv), J_LF_local(6,model.nv);
  J_RF_local.setZero(); J_LF_local.setZero();
  getJointJacobian(model,data_ref,model.getJointId(RF),LOCAL,J_RF_local);
  getJointJacobian(model,data_ref,model.getJointId(LF),LOCAL,J_LF_local);
  
  const SE3 c1Mc2_ref = data_ref.oMi[ci_RF_LF.joint1_id].actInv(data_ref.oMi[ci_RF_LF.joint2_id]);
  
  
  const SE3 rotate(data.oMi[ci_RF_LF.joint1_id].rotation(),SE3::Vector3::Identity());
  J_ref = rotate.toActionMatrix()*(J_RF_local - c1Mc2_ref.toActionMatrix()*J_LF_local);
  
  Eigen::VectorXd rhs_ref(constraint_dim);
  
  rhs_ref.segment<6>(0) = rotate.act(data_ref.a[ci_RF_LF.joint1_id] - c1Mc2_ref.act(data_ref.a[ci_RF_LF.joint2_id])).toVector();
  
  Eigen::MatrixXd KKT_matrix_ref
  = Eigen::MatrixXd::Zero(model.nv+constraint_dim,model.nv+constraint_dim);
  KKT_matrix_ref.bottomRightCorner(model.nv,model.nv) = data_ref.M;
  KKT_matrix_ref.topRightCorner(constraint_dim,model.nv) = J_ref;
  KKT_matrix_ref.bottomLeftCorner(model.nv,constraint_dim) = J_ref.transpose();
  
  forwardDynamics(model,data_ref,q,v,tau,J_ref,rhs_ref,mu0);
  forwardKinematics(model,data_ref,q,v,data_ref.ddq);

  BOOST_CHECK((J_ref*data_ref.ddq+rhs_ref).isZero());
  
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
  BOOST_CHECK((J_ref*data.ddq+rhs_ref).isZero());
  
  BOOST_CHECK(data_ref.ddq.isApprox(data.ddq));
}

BOOST_AUTO_TEST_SUITE_END()
