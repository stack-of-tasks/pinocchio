//
// Copyright(c) 2019 INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_classic_acceleration)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  static const int num_tests = 1e2;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  model.upperPositionLimit.head<3>().fill(100);
  model.upperPositionLimit.segment<4>(3).setOnes();
  model.lowerPositionLimit.head<7>() = - model.upperPositionLimit.head<7>();
  
  VectorXd q(model.nq);
  q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));
  
  Data data_ref(model), data(model);
  
  const std::string RF_joint_name = "rleg6_joint";
  const Model::JointIndex RF_joint_id = model.getJointId(RF_joint_name);

  for(int k = 0; k < num_tests; ++k)
  {
    q = randomConfiguration(model);
    v = VectorXd::Random(model.nv);
    
    forwardKinematics(model,data,q,v,a);
    const SE3 RF_world_transf = SE3(data.oMi[RF_joint_id].rotation(),SE3::Vector3::Zero());
    
    const Motion RF_v_global = RF_world_transf.act(data.v[RF_joint_id]);
    const Motion RF_a_global = RF_world_transf.act(data.a[RF_joint_id]);
    const Motion::Vector3 classic_acc = classicAcceleration(RF_v_global, RF_a_global);
    
    
    Motion::Vector3 classic_acc_other_signature;
    classicAcceleration(RF_v_global, RF_a_global, classic_acc_other_signature);
    BOOST_CHECK(classic_acc_other_signature.isApprox(classic_acc));
    
    // Computes with finite differences
    const double eps = 1e-5;
    const double eps2 = eps * eps;
    forwardKinematics(model,data_ref,q);
    const SE3::Vector3 pos = data_ref.oMi[RF_joint_id].translation();
    
    VectorXd v_plus = v + eps * a;
    VectorXd q_plus = integrate(model,q,v*eps + a*eps2/2.);
    forwardKinematics(model,data_ref,q_plus);
    const SE3::Vector3 pos_plus = data_ref.oMi[RF_joint_id].translation();
    
    VectorXd v_minus = v - eps * a;
    VectorXd q_minus = integrate(model,q,-v*eps - a*eps2/2.);
    forwardKinematics(model,data_ref,q_minus);
    const SE3::Vector3 pos_minus = data_ref.oMi[RF_joint_id].translation();
    
    const SE3::Vector3 classic_acc_ref = (pos_plus + pos_minus - 2.*pos) / eps2;
    
    BOOST_CHECK(classic_acc.isApprox(classic_acc_ref,math::sqrt(eps)));
  }
}

BOOST_AUTO_TEST_CASE(test_classic_acceleration_with_placement)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  static const int num_tests = 1e2;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  model.upperPositionLimit.head<3>().fill(1);
  model.upperPositionLimit.segment<4>(3).setOnes();
  model.lowerPositionLimit.head<7>() = - model.upperPositionLimit.head<7>();
  
  VectorXd q(model.nq);
  q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));
  
  Data data_ref(model), data(model);
  
  const std::string RF_joint_name = "rleg6_joint";
  const Model::JointIndex RF_joint_id = model.getJointId(RF_joint_name);
  
  for(int k = 0; k < num_tests; ++k)
  {
    q = randomConfiguration(model);
    v = VectorXd::Random(model.nv);
    
    forwardKinematics(model,data,q,v,a);
    
    const SE3 RF_world_transf = SE3(data.oMi[RF_joint_id].rotation(),
                                    SE3::Vector3::Zero());
    
    const Motion RF_v_global = RF_world_transf.act(data.v[RF_joint_id]);
    const Motion RF_a_global = RF_world_transf.act(data.a[RF_joint_id]);
    const Motion::Vector3 RF_classic_acc_ref = classicAcceleration(RF_v_global, RF_a_global);
    
    const SE3 identity_placement = SE3::Identity();
    Motion::Vector3 RF_classic_acc = classicAcceleration(RF_v_global, RF_a_global, identity_placement);
    
    BOOST_CHECK(RF_classic_acc.isApprox(RF_classic_acc_ref));
    
    const SE3 random_placement = SE3::Random();
    
    const Motion & v_A = data.v[RF_joint_id];
    const Motion & a_A = data.a[RF_joint_id];
    
    const Motion v_B = random_placement.actInv(data.v[RF_joint_id]);
    const Motion a_B = random_placement.actInv(data.a[RF_joint_id]);
    
    Motion::Vector3 classic_acc_B_ref = classicAcceleration(v_B, a_B);
    Motion::Vector3 classic_acc_B = classicAcceleration(v_A, a_A, random_placement);
    
    BOOST_CHECK(classic_acc_B.isApprox(classic_acc_B_ref));
    
    // Check other signature
    Motion::Vector3 classic_acc_B_other_signature;
    classicAcceleration(v_A, a_A, random_placement, classic_acc_B_other_signature);
    
    BOOST_CHECK(classic_acc_B_other_signature.isApprox(classic_acc_B));
  }
}

BOOST_AUTO_TEST_SUITE_END()
