//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
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

BOOST_AUTO_TEST_CASE ( test_classic_derivatives )
{
  using namespace Eigen;
  using namespace pinocchio;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data(model), data_fd(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill( 1.);
  VectorXd q = randomConfiguration(model);
  
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);
  
  const std::string RF = "rleg6_joint";
  JointIndex RF_id = model.getJointId(RF);
  const std::string LF = "lleg6_joint";
  JointIndex LF_id = model.getJointId(LF);
  
  const double eps = 1e-8;
  
  Data::Matrix6x
  da_dq_fd(6,model.nv),
  da_dv_fd(6,model.nv),
  da_da_fd(6,model.nv);
  
  Data::Matrix6x
  da_dq(6,model.nv),
  da_dv(6,model.nv),
  da_da(6,model.nv);
  
  Data::Matrix6x
  v_partial_dq(Data::Matrix6x::Zero(6,model.nv)),
  a_partial_dq(Data::Matrix6x::Zero(6,model.nv)),
  a_partial_dv(Data::Matrix6x::Zero(6,model.nv)),
  a_partial_da(Data::Matrix6x::Zero(6,model.nv));
  
  const Data::Matrix6x & v_partial_dv = a_partial_da;
  
  computeForwardKinematicsDerivatives(model,data,q,v,a);
  getJointAccelerationDerivatives(model,data,RF_id,WORLD,
                                  v_partial_dq,
                                  a_partial_dq,
                                  a_partial_dv,
                                  a_partial_da);
  
  const Motion & ov_RF = data.ov[RF_id];
  da_dq.middleRows<3>(Motion::LINEAR)
  = a_partial_dq.middleRows<3>(Motion::LINEAR)
  - v_partial_dq.middleRows<3>(Motion::LINEAR).colwise().cross(ov_RF.angular())
  + v_partial_dq.middleRows<3>(Motion::ANGULAR).colwise().cross(ov_RF.linear());
  
  // WORLD : position
  VectorXd q_plus(q), v_eps(VectorXd::Zero(model.nv));
  forwardKinematics(model,data_fd,q,v,a);
  const Data::SE3 oMi_RF = data.oMi[RF_id];
  const Data::Motion v_local_RF = data.v[RF_id];
  const Data::Motion a_local_RF = data.a[RF_id];
  const Motion::Vector3 acc_RF_world = classicAcceleration(oMi_RF.act(v_local_RF),
                                                           oMi_RF.act(a_local_RF));
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_eps[k] = eps;
    q_plus = integrate(model,q,v_eps);
    forwardKinematics(model,data_fd,q_plus,v,a);
    
    const Data::SE3 oMi_RF_plus = data_fd.oMi[RF_id];
    const Data::Motion v_local_RF_plus = data_fd.v[RF_id];
    const Data::Motion a_local_RF_plus = data_fd.a[RF_id];
    const Motion::Vector3 acc_RF_world_plus = classicAcceleration(oMi_RF_plus.act(v_local_RF_plus),
                                                                  oMi_RF_plus.act(a_local_RF_plus));
    da_dq_fd.middleRows<3>(Motion::LINEAR).col(k) = (acc_RF_world_plus - acc_RF_world)/eps;
    
    v_eps[k] = 0;
  }
  
  BOOST_CHECK(da_dq.middleRows<3>(Motion::LINEAR).isApprox(da_dq_fd.middleRows<3>(Motion::LINEAR),sqrt(eps)));
  
  // WORLD : velocity
  da_dv.middleRows<3>(Motion::LINEAR)
  = a_partial_dv.middleRows<3>(Motion::LINEAR)
  - v_partial_dv.middleRows<3>(Motion::LINEAR).colwise().cross(ov_RF.angular())
  + v_partial_dv.middleRows<3>(Motion::ANGULAR).colwise().cross(ov_RF.linear());
  
  VectorXd v_plus(v);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    v_plus = v;
    v_plus[k] += eps;
    forwardKinematics(model,data_fd,q,v_plus,a);
    
    const Data::SE3 oMi_RF_plus = data_fd.oMi[RF_id];
    const Data::Motion v_local_RF_plus = data_fd.v[RF_id];
    const Data::Motion a_local_RF_plus = data_fd.a[RF_id];
    const Motion::Vector3 acc_RF_world_plus = classicAcceleration(oMi_RF_plus.act(v_local_RF_plus),
                                                                  oMi_RF_plus.act(a_local_RF_plus));
    da_dv_fd.middleRows<3>(Motion::LINEAR).col(k) = (acc_RF_world_plus - acc_RF_world)/eps;
  }
  
  BOOST_CHECK(da_dv.middleRows<3>(Motion::LINEAR).isApprox(da_dv_fd.middleRows<3>(Motion::LINEAR),sqrt(eps)));
  
  // WORLD : acceleration
  da_da.middleRows<3>(Motion::LINEAR)
  = a_partial_da.middleRows<3>(Motion::LINEAR);
  
  VectorXd a_plus(a);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    a_plus = a;
    a_plus[k] += eps;
    forwardKinematics(model,data_fd,q,v,a_plus);
    
    const Data::SE3 oMi_RF_plus = data_fd.oMi[RF_id];
    const Data::Motion v_local_RF_plus = data_fd.v[RF_id];
    const Data::Motion a_local_RF_plus = data_fd.a[RF_id];
    const Motion::Vector3 acc_RF_world_plus = classicAcceleration(oMi_RF_plus.act(v_local_RF_plus),
                                                                  oMi_RF_plus.act(a_local_RF_plus));
    da_da_fd.middleRows<3>(Motion::LINEAR).col(k) = (acc_RF_world_plus - acc_RF_world)/eps;
  }
  
  BOOST_CHECK(da_da.middleRows<3>(Motion::LINEAR).isApprox(da_da_fd.middleRows<3>(Motion::LINEAR),sqrt(eps)));
  
}

BOOST_AUTO_TEST_SUITE_END ()

