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


BOOST_AUTO_TEST_CASE(test_correction_6D)
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
  const double mu = 0.;
  ProximalSettings prox_settings(1e-12,mu,1);
  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  const Model::JointIndex LF_id = model.getJointId(LF);

  // Contact models and data
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_datas,   contact_datas_fd;
  
  
  RigidConstraintModel ci_RF(CONTACT_6D,RF_id,LOCAL);
  ci_RF.joint1_placement.setRandom();
  ci_RF.joint2_placement.setRandom();
  ci_RF.corrector.Kp = 10.;
  ci_RF.corrector.Kd = 2. * sqrt(ci_RF.corrector.Kp);
  contact_models.push_back(ci_RF);
  contact_datas.push_back(RigidConstraintData(ci_RF));
  contact_datas_fd.push_back(RigidConstraintData(ci_RF));
    
  RigidConstraintModel ci_LF(CONTACT_3D,LF_id,LOCAL);
  ci_LF.joint1_placement.setRandom();
  ci_LF.joint2_placement.setRandom();
  ci_LF.corrector.Kp = 10.;
  ci_LF.corrector.Kd = 2. * sqrt(ci_LF.corrector.Kp);
  contact_models.push_back(ci_LF);
  contact_datas.push_back(RigidConstraintData(ci_LF));
  contact_datas_fd.push_back(RigidConstraintData(ci_LF));
  
  Eigen::DenseIndex constraint_dim = 0;
  for(size_t k = 0; k < contact_models.size(); ++k)
    constraint_dim += contact_models[k].size();
  
  initConstraintDynamics(model,data,contact_models);
  const Eigen::VectorXd ddq0 = constraintDynamics(model,data,q,v,tau,contact_models,contact_datas,prox_settings);
  
  Eigen::MatrixXd ddq_dq(model.nv,model.nv), ddq_dv(model.nv,model.nv), ddq_dtau(model.nv,model.nv);
  Eigen::MatrixXd dlambda_dq(constraint_dim,model.nv), dlambda_dv(constraint_dim,model.nv), dlambda_dtau(constraint_dim,model.nv);
  
  computeConstraintDynamicsDerivatives(model,data,contact_models,contact_datas,
                                    ddq_dq,ddq_dv,ddq_dtau,
                                    dlambda_dq,dlambda_dv,dlambda_dtau);
  computeForwardKinematicsDerivatives(model,data,q,v,0*v);
  
  Data::Matrix6x dv_RF_dq_L(Data::Matrix6x::Zero(6,model.nv));
  Data::Matrix6x dv_RF_dv_L(Data::Matrix6x::Zero(6,model.nv));
  getFrameVelocityDerivatives(model,data,ci_RF.joint1_id,ci_RF.joint1_placement,ci_RF.reference_frame,dv_RF_dq_L,dv_RF_dv_L);
  
  Data::Matrix6x dv_LF_dq_L(Data::Matrix6x::Zero(6,model.nv));
  Data::Matrix6x dv_LF_dv_L(Data::Matrix6x::Zero(6,model.nv));
  getFrameVelocityDerivatives(model,data,ci_LF.joint1_id,ci_LF.joint1_placement,ci_LF.reference_frame,dv_LF_dq_L,dv_LF_dv_L);
  
  const double eps = 1e-8;
  Data::Matrix6x dacc_corrector_RF_dq(6,model.nv); dacc_corrector_RF_dq.setZero();
  Data::Matrix6x dacc_corrector_RF_dv(6,model.nv); dacc_corrector_RF_dv.setZero();
  Data::Matrix3x dacc_corrector_LF_dq(3,model.nv); dacc_corrector_LF_dq.setZero();
  Data::Matrix3x dacc_corrector_LF_dv(3,model.nv); dacc_corrector_LF_dv.setZero();

  {
    const SE3::Matrix6 Jlog = Jlog6(contact_datas[0].c1Mc2.inverse());
    dacc_corrector_RF_dq =  - ci_RF.corrector.Kp * Jlog * dv_RF_dv_L;
    dacc_corrector_RF_dq += - ci_RF.corrector.Kd * dv_RF_dq_L;
    
    dacc_corrector_RF_dv = - ci_RF.corrector.Kd * dv_RF_dv_L;
    BOOST_CHECK(dv_RF_dv_L.isApprox(data.contact_chol.matrix().topRightCorner(6,model.nv)));
  }
  
  {
    dacc_corrector_LF_dq =  - ci_LF.corrector.Kp * dv_LF_dv_L.topRows<3>();
    dacc_corrector_LF_dq += - ci_LF.corrector.Kd * dv_LF_dq_L.topRows<3>();
    for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
    {
      dacc_corrector_LF_dq.col(k) += ci_LF.corrector.Kp * dv_LF_dv_L.col(k).tail<3>().cross(contact_datas[1].contact_placement_error.linear());
    }
    
    dacc_corrector_LF_dv = - ci_LF.corrector.Kd * dv_LF_dv_L.topRows<3>();
    BOOST_CHECK(dv_LF_dv_L.topRows<3>().isApprox(data.contact_chol.matrix().topRightCorner(9,model.nv).bottomRows<3>()));
  }
  
  initConstraintDynamics(model,data_fd,contact_models);
  
  Data::Matrix6x dacc_corrector_RF_dq_fd(6,model.nv);
  Data::Matrix3x dacc_corrector_LF_dq_fd(3,model.nv);
  
  Eigen::MatrixXd ddq_dq_fd(model.nv,model.nv), ddq_dv_fd(model.nv,model.nv), ddq_dtau_fd(model.nv,model.nv);
  Eigen::MatrixXd dlambda_dq_fd(constraint_dim,model.nv), dlambda_dv_fd(constraint_dim,model.nv), dlambda_dtau_fd(constraint_dim,model.nv);
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_eps = Eigen::VectorXd::Zero(model.nv);
    v_eps[k] = eps;
    const Eigen::VectorXd q_plus = integrate(model,q,v_eps);
    
    Eigen::VectorXd ddq_plus = constraintDynamics(model,data_fd,q_plus,v,tau,contact_models,contact_datas_fd,prox_settings);
    dacc_corrector_RF_dq_fd.col(k) = (contact_datas_fd[0].contact_acceleration_error - contact_datas[0].contact_acceleration_error).toVector() / eps;
    dacc_corrector_LF_dq_fd.col(k) = (contact_datas_fd[1].contact_acceleration_error.linear() - contact_datas[1].contact_acceleration_error.linear()) / eps;
    
    ddq_dq_fd.col(k) = (ddq_plus - ddq0)/eps;
  }
  
  BOOST_CHECK(ddq_dq_fd.isApprox(ddq_dq,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dq.isApprox(dacc_corrector_RF_dq_fd,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dq.isApprox(dacc_corrector_LF_dq_fd,sqrt(eps)));
  // std::cout << "dacc_corrector_RF_dq:\n" << dacc_corrector_RF_dq << std::endl;
  // std::cout << "dacc_corrector_RF_dq_fd:\n" << dacc_corrector_RF_dq_fd << std::endl;
  
  Data::Matrix6x dacc_corrector_RF_dv_fd(6,model.nv);
  Data::Matrix3x dacc_corrector_LF_dv_fd(3,model.nv);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    Eigen::VectorXd v_plus(v);
    v_plus[k] += eps;
    
    Eigen::VectorXd ddq_plus = constraintDynamics(model,data_fd,q,v_plus,tau,contact_models,contact_datas_fd,prox_settings);
    dacc_corrector_RF_dv_fd.col(k) = (contact_datas_fd[0].contact_acceleration_error - contact_datas[0].contact_acceleration_error).toVector() / eps;
    dacc_corrector_LF_dv_fd.col(k) = (contact_datas_fd[1].contact_acceleration_error.linear() - contact_datas[1].contact_acceleration_error.linear()) / eps;
    
    ddq_dv_fd.col(k) = (ddq_plus - ddq0)/eps;
  }
  BOOST_CHECK(ddq_dv_fd.isApprox(ddq_dv,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_RF_dv.isApprox(dacc_corrector_RF_dv_fd,sqrt(eps)));
  BOOST_CHECK(dacc_corrector_LF_dv.isApprox(dacc_corrector_LF_dv_fd,sqrt(eps)));
}

BOOST_AUTO_TEST_SUITE_END ()
