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

pinocchio::Motion computeAcceleration(const pinocchio::Model & model,
                                      const pinocchio::Data & data,
                                      const pinocchio::JointIndex & joint_id,
                                      const pinocchio::ReferenceFrame reference_frame,
                                      const pinocchio::ContactType contact_type,
                                      const pinocchio::SE3 & placement = pinocchio::SE3::Identity())
{
  PINOCCHIO_UNUSED_VARIABLE(model);
  using namespace pinocchio;
  Motion res(Motion::Zero());

  const Data::SE3 & oMi = data.oMi[joint_id];
  const Data::SE3 oMc = oMi * placement;

  const Data::SE3 & iMc = placement;
  const Motion ov = oMi.act(data.v[joint_id]);
  const Motion oa = oMi.act(data.a[joint_id]);

  switch (reference_frame)
  {
    case WORLD:
      if(contact_type == CONTACT_6D)
        return oa;
      classicAcceleration(ov,oa,res.linear());
      res.angular() = oa.angular();
      break;
    case LOCAL_WORLD_ALIGNED:
      if(contact_type == CONTACT_6D)
      {
        res.linear() = oMc.rotation() * iMc.actInv(data.a[joint_id]).linear();
        res.angular() = oMi.rotation() * data.a[joint_id].angular();
      }
      else
      {
        res.linear() = oMc.rotation() * classicAcceleration(data.v[joint_id],data.a[joint_id],iMc);
        res.angular() = oMi.rotation() * data.a[joint_id].angular();
      }
      break;
    case LOCAL:
      if(contact_type == CONTACT_6D)
        return oMc.actInv(oa);
      classicAcceleration(data.v[joint_id],data.a[joint_id],iMc,res.linear());
      res.angular() = iMc.rotation().transpose() * data.a[joint_id].angular();
      break;
    default:
      break;
  }

  return res;
}

pinocchio::Motion getContactAcceleration(const Model & model,
                                         const Data & data,
                                         const RigidConstraintModel & cmodel)
{
  return computeAcceleration(model,data,cmodel.joint1_id,cmodel.reference_frame,cmodel.type,cmodel.joint1_placement);
}


BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_mix_fd)
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
  const Model::JointIndex LF_id = model.getJointId(LF);
  const std::string RH = "rarm6_joint";
  const Model::JointIndex RH_id = model.getJointId(RH);
  const std::string LH = "larm6_joint";
  const Model::JointIndex LH_id = model.getJointId(LH);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D,model,LF_id,LOCAL_WORLD_ALIGNED);
  ci_LF.joint1_placement.setRandom();
  contact_models.push_back(ci_LF); contact_data.push_back(RigidConstraintData(ci_LF));
  RigidConstraintModel ci_RF(CONTACT_6D,model,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  contact_models.push_back(ci_RF); contact_data.push_back(RigidConstraintData(ci_RF));
  
  RigidConstraintModel ci_LH(CONTACT_3D,model,LH_id,LOCAL_WORLD_ALIGNED);
  ci_LH.joint1_placement.setRandom();
  contact_models.push_back(ci_LH); contact_data.push_back(RigidConstraintData(ci_LH));
  RigidConstraintModel ci_RH(CONTACT_3D,model,RH_id,LOCAL);
  ci_RH.joint1_placement.setRandom();
  contact_models.push_back(ci_RH); contact_data.push_back(RigidConstraintData(ci_RH));

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
  
  const Eigen::MatrixXd Jc = data.dac_da;
  const Eigen::MatrixXd Jc_ref = data.contact_chol.matrix().topRightCorner(constraint_dim,model.nv);
  
  BOOST_CHECK(Jc.isApprox(Jc_ref));
  
  const Eigen::MatrixXd JMinv = Jc * data.Minv;
  const Eigen::MatrixXd dac_dq = data.dac_dq + JMinv * data.dtau_dq;
  
  Eigen::MatrixXd dac_dq_fd(constraint_dim,model.nv);
  
  Eigen::VectorXd contact_acc0(constraint_dim);
  Eigen::DenseIndex row_id = 0;
  
  forwardKinematics(model,data,q,v,data.ddq);
  for(size_t k = 0; k < contact_models.size(); ++k)
  {
    const RigidConstraintModel & cmodel = contact_models[k];
    const Eigen::DenseIndex size = cmodel.size();
    
    const Motion contact_acc = getContactAcceleration(model,data,cmodel);
    
    if(cmodel.type == CONTACT_3D)
      contact_acc0.segment<3>(row_id) = contact_acc.linear();
    else
      contact_acc0.segment<6>(row_id) = contact_acc.toVector();
    
    row_id += size;
  }
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] += alpha;
    q_plus = integrate(model,q,v_eps);
    ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_data,prox_settings);
    
    ddq_partial_dq_fd.col(k) = (ddq_plus - ddq0)/alpha;
    lambda_partial_dq_fd.col(k) = (data_fd.lambda_c - lambda0)/alpha;
    
    Eigen::VectorXd contact_acc_plus(constraint_dim);
    Eigen::DenseIndex row_id = 0;
    forwardKinematics(model,data_fd,q_plus,v,data.ddq);
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const Eigen::DenseIndex size = cmodel.size();
      
      const Motion contact_acc = getContactAcceleration(model,data_fd,cmodel);
      
      if(cmodel.type == CONTACT_3D)
        contact_acc_plus.segment<3>(row_id) = contact_acc.linear();
      else
        contact_acc_plus.segment<6>(row_id) = contact_acc.toVector();
      
      row_id += size;
    }
    
    dac_dq_fd.col(k) = (contact_acc_plus - contact_acc0)/alpha;
    
    v_eps[k] = 0.;
  }
  
  // std::cout << "dac_dq_fd:\n" << dac_dq_fd << std::endl;
  // std::cout << "dac_dq:\n" << data.dac_da << std::endl;
  // std::cout << "dac_dq:\n" << dac_dq_fd<< std::endl;

  BOOST_CHECK(ddq_partial_dq_fd.isApprox(data.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(lambda_partial_dq_fd.isApprox(data.dlambda_dq,sqrt(alpha)));
  
  // std::cout << "lambda_partial_dq_fd:\n" << lambda_partial_dq_fd << std::endl;
  // std::cout << "dlambda_dq:\n" << data.dlambda_dq << std::endl;

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

BOOST_AUTO_TEST_CASE(test_constraint_dynamics_derivatives_dirty_data)
{
  // Verify that a dirty data doesn't affect the results of the contact dynamics derivs
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model,true);
  Data data_dirty(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);

  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;

  RigidConstraintModel ci_LF(CONTACT_6D,model,LF_id,LOCAL);
  RigidConstraintModel ci_RF(CONTACT_3D,model,RF_id,LOCAL);

  contact_models.push_back(ci_LF); contact_data.push_back(RigidConstraintData(ci_LF));
  contact_models.push_back(ci_RF); contact_data.push_back(RigidConstraintData(ci_RF));

  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();

  const double mu0 = 0.;
  ProximalSettings prox_settings(1e-12,mu0,1);
  initConstraintDynamics(model,data_dirty,contact_models);
  constraintDynamics(model,data_dirty,q,v,tau,contact_models,contact_data,prox_settings);
  computeConstraintDynamicsDerivatives(model, data_dirty, contact_models, contact_data);

  // Reuse the same data with new configurations
  q = randomConfiguration(model);
  v = VectorXd::Random(model.nv);
  tau = VectorXd::Random(model.nv);
  constraintDynamics(model,data_dirty,q,v,tau,contact_models,contact_data,prox_settings);
  computeConstraintDynamicsDerivatives(model, data_dirty, contact_models, contact_data);

  //Test with fresh data
  Data data_fresh(model);
  initConstraintDynamics(model,data_fresh,contact_models);
  constraintDynamics(model,data_fresh,q,v,tau,contact_models,contact_data,prox_settings);
  computeConstraintDynamicsDerivatives(model, data_fresh, contact_models, contact_data);
  const double alpha = 1e-12;

  BOOST_CHECK(data_dirty.ddq_dq.isApprox(data_fresh.ddq_dq,sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dv.isApprox(data_fresh.ddq_dv,sqrt(alpha)));
  BOOST_CHECK(data_dirty.ddq_dtau.isApprox(data_fresh.ddq_dtau,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dq.isApprox(data_fresh.dlambda_dq,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dv.isApprox(data_fresh.dlambda_dv,sqrt(alpha)));
  BOOST_CHECK(data_dirty.dlambda_dtau.isApprox(data_fresh.dlambda_dtau,sqrt(alpha)));
}

BOOST_AUTO_TEST_SUITE_END ()
