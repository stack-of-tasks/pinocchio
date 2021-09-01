//
// Copyright (c) 2020 CNRS INRIA
//

#include <iostream>

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/constrained-dynamics.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_RF(CONTACT_3D,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initConstraintDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_loop_closure_j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure (CONTACT_3D, 0, SE3::Identity(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initConstraintDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_3D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure (CONTACT_3D, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initConstraintDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_loop_closure_j1j2_fd)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
    const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  // Add Loop Closure Constraint
  
  const std::string RA = "rarm5_joint";
  const Model::JointIndex RA_id = model.getJointId(RA);
  const std::string LA = "larm5_joint";
  const Model::JointIndex LA_id = model.getJointId(LA);

  RigidConstraintModel ci_closure (CONTACT_3D, LA_id, SE3::Random(),
                                RA_id, SE3::Random(), LOCAL_WORLD_ALIGNED);
  
  contact_models.push_back(ci_closure);
  contact_data.push_back(RigidConstraintData(ci_closure));
  // End of Loopo Closure Constraint

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);
  const Data::TangentVectorType a = data.ddq;
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initConstraintDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  forwardKinematics(model,data,q,v,a);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }
  
  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_CASE ( test_constraint_dynamics_derivatives_LOCAL_WORLD_ALIGNED_3D_fd )
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data(model), data_fd(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  //  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_RF(CONTACT_3D,RF_id,LOCAL_WORLD_ALIGNED);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data,contact_models);
  constraintDynamics(model,data,q,v,tau,contact_models,contact_data,prox_settings);

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  computeConstraintDynamicsDerivatives(model, data, contact_models, contact_data);

  //Data_fd
  initConstraintDynamics(model,data_fd,contact_models);

  MatrixXd ddq_partial_dq_fd(model.nv,model.nv); ddq_partial_dq_fd.setZero();
  MatrixXd ddq_partial_dv_fd(model.nv,model.nv); ddq_partial_dv_fd.setZero();
  MatrixXd ddq_partial_dtau_fd(model.nv,model.nv); ddq_partial_dtau_fd.setZero();

  MatrixXd lambda_partial_dtau_fd(constraint_dim,model.nv); lambda_partial_dtau_fd.setZero();
  MatrixXd lambda_partial_dq_fd(constraint_dim,model.nv); lambda_partial_dq_fd.setZero();
  MatrixXd lambda_partial_dv_fd(constraint_dim,model.nv); lambda_partial_dv_fd.setZero();

  const VectorXd ddq0 = constraintDynamics(model,data_fd,q,v,tau,contact_models,contact_data,prox_settings);
  const VectorXd lambda0 = data_fd.lambda_c;
  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd ddq_plus(model.nv);

  VectorXd lambda_plus(constraint_dim);

  const double alpha = 1e-8;
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_eps[k] = 0.;
  }

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));

  VectorXd v_plus(v);
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_data,prox_settings);
    ddq_partial_dv_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dv_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    v_plus[k] -= alpha;
  }

  BOOST_CHECK(ddq_partial_dv_fd.isApprox(data.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dv_fd.isApprox(data.dlambda_dv,sqrt(alpha)));

  VectorXd tau_plus(tau);
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += alpha;
    ddq_plus = constraintDynamics(model,data_fd,q,v,tau_plus,contact_models,contact_data,prox_settings);
    ddq_partial_dtau_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dtau_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    tau_plus[k] -= alpha;
  }

  BOOST_CHECK(lambda_partial_dtau_fd.isApprox(data.dlambda_dtau,sqrt(alpha)));
  BOOST_CHECK(ddq_partial_dtau_fd.isApprox(data.ddq_dtau,sqrt(alpha)));
}


BOOST_AUTO_TEST_SUITE_END ()
