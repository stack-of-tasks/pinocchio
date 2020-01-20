//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

using namespace Eigen;
using namespace pinocchio;

BOOST_AUTO_TEST_CASE ( test_FD_with_contact_cst_gamma )
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_check(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  normalize(model,q);

  computeJointJacobians(model, data, q);
  
  VectorXd v = VectorXd::Ones(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
//  const std::string LF = "lleg6_joint";
//  const Model::JointIndex LF_id = model.getJointId(LF);
  
  Data::Matrix6x J_RF(6,model.nv); J_RF.setZero();
  getJointJacobian(model, data, RF_id, LOCAL, J_RF);
  Motion::Vector6 gamma_RF; gamma_RF.setZero();
  forwardKinematics(model,data,q,v,VectorXd::Zero(model.nv));
  gamma_RF += data.a[RF_id].toVector(); // Jdot * qdot
  
  forwardDynamics(model, data, q, v, tau, J_RF, gamma_RF);
  VectorXd ddq_ref = data.ddq;
  Force::Vector6 contact_force_ref = data.lambda_c;
  
  PINOCCHIO_ALIGNED_STD_VECTOR(Force) fext((size_t)model.njoints,Force::Zero());
  fext[RF_id] = ForceRef<Force::Vector6>(contact_force_ref);
  
  // check call to RNEA
  rnea(model,data_check,q,v,ddq_ref,fext);
  
  BOOST_CHECK(data_check.tau.isApprox(tau));
  forwardKinematics(model,data_check,q,VectorXd::Zero(model.nv),ddq_ref);
  BOOST_CHECK(data_check.a[RF_id].toVector().isApprox(-gamma_RF));
  
  Data data_fd(model);
  VectorXd q_plus(model.nq);
  VectorXd v_eps(model.nv); v_eps.setZero();
  VectorXd v_plus(v);
  VectorXd tau_plus(tau);
  const double eps = 1e-8;
  
  // check: dddq_dtau and dlambda_dtau
  MatrixXd dddq_dtau(model.nv,model.nv);
  Data::Matrix6x dlambda_dtau(6,model.nv);
  
  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += eps;
    forwardDynamics(model, data_fd, q, v, tau_plus, J_RF, gamma_RF);
    
    const Data::TangentVectorType & ddq_plus = data_fd.ddq;
    Force::Vector6 contact_force_plus = data_fd.lambda_c;
    
    dddq_dtau.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dtau.col(k) = (contact_force_plus - contact_force_ref)/eps;
    
    tau_plus[k] -= eps;
  }
    
  MatrixXd A(model.nv+6,model.nv+6);
  data.M.transpose().triangularView<Eigen::Upper>() = data.M.triangularView<Eigen::Upper>();
  A.topLeftCorner(model.nv,model.nv) = data.M;
  A.bottomLeftCorner(6, model.nv) = J_RF;
  A.topRightCorner(model.nv, 6) = J_RF.transpose();
  A.bottomRightCorner(6,6).setZero();
  
  MatrixXd Ainv = A.inverse();
  BOOST_CHECK(Ainv.topRows(model.nv).leftCols(model.nv).isApprox(dddq_dtau,std::sqrt(eps)));
  BOOST_CHECK(Ainv.bottomRows(6).leftCols(model.nv).isApprox(-dlambda_dtau,std::sqrt(eps)));
  
  // check: dddq_dv and dlambda_dv
  MatrixXd dddq_dv(model.nv,model.nv);
  Data::Matrix6x dlambda_dv(6,model.nv);
  
  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += eps;
    forwardDynamics(model, data_fd, q, v_plus, tau, J_RF, gamma_RF);
    
    const Data::TangentVectorType & ddq_plus = data_fd.ddq;
    Force::Vector6 contact_force_plus = data_fd.lambda_c;
    
    dddq_dv.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dv.col(k) = (contact_force_plus - contact_force_ref)/eps;
    
    v_plus[k] -= eps;
  }
  
  computeRNEADerivatives(model,data_check,q,v,VectorXd::Zero(model.nv));
  MatrixXd dddq_dv_anal = -Ainv.topRows(model.nv).leftCols(model.nv) * data_check.dtau_dv;
  MatrixXd dlambda_dv_anal = -Ainv.bottomRows(6).leftCols(model.nv) * data_check.dtau_dv;
  
  BOOST_CHECK(dddq_dv_anal.isApprox(dddq_dv,std::sqrt(eps)));
  BOOST_CHECK(dlambda_dv_anal.isApprox(-dlambda_dv,std::sqrt(eps)));
  
  MatrixXd dddq_dq(model.nv,model.nv);
  Data::Matrix6x dlambda_dq(6,model.nv);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = integrate(model,q,v_eps);
    computeJointJacobians(model, data_fd, q_plus);
    getJointJacobian(model, data_fd, RF_id, LOCAL, J_RF);
    forwardDynamics(model, data_fd, q_plus, v, tau, J_RF, gamma_RF);
    
    const Data::TangentVectorType & ddq_plus = data_fd.ddq;
    Force::Vector6 contact_force_plus = data_fd.lambda_c;
    
    dddq_dq.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dq.col(k) = (contact_force_plus - contact_force_ref)/eps;
    
    v_eps[k] = 0.;
  }
  
  computeRNEADerivatives(model,data_check,q,v,ddq_ref,fext);
  Data::Matrix6x v_partial_dq(6,model.nv), a_partial_dq(6,model.nv), a_partial_dv(6,model.nv), a_partial_da(6,model.nv);
  v_partial_dq.setZero(); a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  Data data_kin(model);
  computeForwardKinematicsDerivatives(model,data_kin,q,VectorXd::Zero(model.nv),ddq_ref);
  getJointAccelerationDerivatives(model,data_kin,RF_id,LOCAL,v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);
  
  MatrixXd dddq_dq_anal = -Ainv.topRows(model.nv).leftCols(model.nv) * data_check.dtau_dq;
  dddq_dq_anal -= Ainv.topRows(model.nv).rightCols(6) * a_partial_dq;
  
  MatrixXd dlambda_dq_anal = Ainv.bottomRows(6).leftCols(model.nv) * data_check.dtau_dq;
  dlambda_dq_anal += Ainv.bottomRows(6).rightCols(6) * a_partial_dq;
  
  BOOST_CHECK(dddq_dq_anal.isApprox(dddq_dq,std::sqrt(eps)));
  BOOST_CHECK(dlambda_dq_anal.isApprox(dlambda_dq,std::sqrt(eps)));
  
}

template<typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
VectorXd contactDynamics(const Model & model, Data & data,
                         const Eigen::MatrixBase<ConfigVectorType> & q,
                         const Eigen::MatrixBase<TangentVectorType1> & v,
                         const Eigen::MatrixBase<TangentVectorType2> & tau,
                         const Model::JointIndex id)
{
  computeJointJacobians(model, data, q);
  Data::Matrix6x J(6,model.nv); J.setZero();
  
  getJointJacobian(model, data, id, LOCAL, J);
  Motion::Vector6 gamma;
  forwardKinematics(model, data, q, v, VectorXd::Zero(model.nv));
  gamma = data.a[id].toVector();
  
  forwardDynamics(model, data, q, v, tau, J, gamma);
  VectorXd res(VectorXd::Zero(model.nv+6));
  
  res.head(model.nv) = data.ddq;
  res.tail(6) = data.lambda_c;
  
  return res;
}

BOOST_AUTO_TEST_CASE ( test_FD_with_contact_varying_gamma )
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_check(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  normalize(model,q);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd tau = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  const Model::JointIndex RF_id = model.getJointId(RF);
  
  Data::Matrix6x J_RF(6,model.nv); J_RF.setZero();
  computeJointJacobians(model, data, q);
  getJointJacobian(model, data, RF_id, LOCAL, J_RF);
  Motion::Vector6 gamma_RF; gamma_RF.setZero();
  
  VectorXd x_ref = contactDynamics(model,data,q,v,tau,RF_id);
  VectorXd ddq_ref = x_ref.head(model.nv);
  Force::Vector6 contact_force_ref = x_ref.tail(6);
  
  PINOCCHIO_ALIGNED_STD_VECTOR(Force) fext((size_t)model.njoints,Force::Zero());
  fext[RF_id] = ForceRef<Force::Vector6>(contact_force_ref);
  
  // check call to RNEA
  rnea(model,data_check,q,v,ddq_ref,fext);
  
  BOOST_CHECK(data_check.tau.isApprox(tau));
  forwardKinematics(model,data_check,q,v,ddq_ref);
  BOOST_CHECK(data_check.a[RF_id].toVector().isZero());
  
  Data data_fd(model);
  VectorXd q_plus(model.nq);
  VectorXd v_eps(model.nv); v_eps.setZero();
  VectorXd v_plus(v);
  VectorXd tau_plus(tau);
  VectorXd x_plus(model.nv + 6);
  const double eps = 1e-8;

  // check: dddq_dtau and dlambda_dtau
  MatrixXd dddq_dtau(model.nv,model.nv);
  Data::Matrix6x dlambda_dtau(6,model.nv);

  for(int k = 0; k < model.nv; ++k)
  {
    tau_plus[k] += eps;
    x_plus = contactDynamics(model,data,q,v,tau_plus,RF_id);

    const Data::TangentVectorType ddq_plus = x_plus.head(model.nv);
    Force::Vector6 contact_force_plus = x_plus.tail(6);

    dddq_dtau.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dtau.col(k) = (contact_force_plus - contact_force_ref)/eps;

    tau_plus[k] -= eps;
  }

  MatrixXd A(model.nv+6,model.nv+6);
  data.M.transpose().triangularView<Eigen::Upper>() = data.M.triangularView<Eigen::Upper>();
  A.topLeftCorner(model.nv,model.nv) = data.M;
  A.bottomLeftCorner(6, model.nv) = J_RF;
  A.topRightCorner(model.nv, 6) = J_RF.transpose();
  A.bottomRightCorner(6,6).setZero();

  MatrixXd Ainv = A.inverse();
  BOOST_CHECK(Ainv.topRows(model.nv).leftCols(model.nv).isApprox(dddq_dtau,std::sqrt(eps)));
  BOOST_CHECK(Ainv.bottomRows(6).leftCols(model.nv).isApprox(-dlambda_dtau,std::sqrt(eps)));

  // check: dddq_dv and dlambda_dv
  MatrixXd dddq_dv(model.nv,model.nv);
  Data::Matrix6x dlambda_dv(6,model.nv);

  for(int k = 0; k < model.nv; ++k)
  {
    v_plus[k] += eps;
    x_plus = contactDynamics(model,data,q,v_plus,tau,RF_id);

    const Data::TangentVectorType ddq_plus = x_plus.head(model.nv);
    Force::Vector6 contact_force_plus = x_plus.tail(6);

    dddq_dv.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dv.col(k) = (contact_force_plus - contact_force_ref)/eps;

    v_plus[k] -= eps;
  }
  
  
  computeRNEADerivatives(model,data_check,q,v,VectorXd::Zero(model.nv));
  Data::Matrix6x v_partial_dq(6,model.nv), a_partial_dq(6,model.nv), a_partial_dv(6,model.nv), a_partial_da(6,model.nv);
  v_partial_dq.setZero(); a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  Data data_kin(model);
  computeForwardKinematicsDerivatives(model,data_kin,q,v,VectorXd::Zero(model.nv));
  getJointAccelerationDerivatives(model,data_kin,RF_id,LOCAL,v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);
  
  MatrixXd dddq_dv_anal = -Ainv.topRows(model.nv).leftCols(model.nv) * data_check.dtau_dv;
  dddq_dv_anal -= Ainv.topRows(model.nv).rightCols(6) * a_partial_dv;
  MatrixXd dlambda_dv_anal = -Ainv.bottomRows(6).leftCols(model.nv) * data_check.dtau_dv;
  dlambda_dv_anal -= Ainv.bottomRows(6).rightCols(6) * a_partial_dv;
  
  BOOST_CHECK(dddq_dv_anal.isApprox(dddq_dv,std::sqrt(eps)));
  BOOST_CHECK(dlambda_dv_anal.isApprox(-dlambda_dv,std::sqrt(eps)));
  

  MatrixXd dddq_dq(model.nv,model.nv);
  Data::Matrix6x dlambda_dq(6,model.nv);

  for(int k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = integrate(model,q,v_eps);
    
    x_plus = contactDynamics(model,data,q_plus,v,tau,RF_id);
    
    const Data::TangentVectorType ddq_plus = x_plus.head(model.nv);
    Force::Vector6 contact_force_plus = x_plus.tail(6);

    dddq_dq.col(k) = (ddq_plus - ddq_ref)/eps;
    dlambda_dq.col(k) = (contact_force_plus - contact_force_ref)/eps;

    v_eps[k] = 0.;
  }

  computeRNEADerivatives(model,data_check,q,v,ddq_ref,fext);
  v_partial_dq.setZero(); a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  computeForwardKinematicsDerivatives(model,data_kin,q,v,ddq_ref);
  getJointAccelerationDerivatives(model,data_kin,RF_id,LOCAL,v_partial_dq,a_partial_dq,a_partial_dv,a_partial_da);

  MatrixXd dddq_dq_anal = -Ainv.topRows(model.nv).leftCols(model.nv) * data_check.dtau_dq;
  dddq_dq_anal -= Ainv.topRows(model.nv).rightCols(6) * a_partial_dq;

  BOOST_CHECK(dddq_dq_anal.isApprox(dddq_dq,std::sqrt(eps)));
  
  MatrixXd dlambda_dq_anal = Ainv.bottomRows(6).leftCols(model.nv) * data_check.dtau_dq;
  dlambda_dq_anal += Ainv.bottomRows(6).rightCols(6) * a_partial_dq;
  
  BOOST_CHECK(dlambda_dq_anal.isApprox(dlambda_dq,std::sqrt(eps)));
  
}

BOOST_AUTO_TEST_SUITE_END ()

