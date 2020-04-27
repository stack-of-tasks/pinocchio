//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_kinematics_constant_vector_input)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  forwardKinematics(model,data,Model::ConfigVectorType::Ones(model.nq));
}

BOOST_AUTO_TEST_CASE(test_kinematics_zero)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  forwardKinematics(model,data_ref,q);
  crba(model,data,q);
  updateGlobalPlacements(model,data);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.oMi[i] == data_ref.oMi[i]);
    BOOST_CHECK(data.liMi[i] == data_ref.liMi[i]);
  }
}

BOOST_AUTO_TEST_CASE(test_kinematics_first)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  
  forwardKinematics(model,data,q,v);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.v[i] == Motion::Zero());
  }
}

BOOST_AUTO_TEST_CASE(test_kinematics_second)
{
  using namespace Eigen;
  using namespace pinocchio;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  Data data(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Zero(model.nv));
  VectorXd a(VectorXd::Zero(model.nv));
  
  forwardKinematics(model,data,q,v,a);
  
  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.v[i] == Motion::Zero());
    BOOST_CHECK(data.a[i] == Motion::Zero());
  }
}

BOOST_AUTO_TEST_CASE(test_get_velocity)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));

  forwardKinematics(model,data,q,v);

  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.v[i].isApprox(getVelocity(model,data,i)));
    BOOST_CHECK(data.v[i].isApprox(getVelocity(model,data,i,LOCAL)));
    BOOST_CHECK(data.oMi[i].act(data.v[i]).isApprox(getVelocity(model,data,i,WORLD)));
    BOOST_CHECK(SE3(data.oMi[i].rotation(), Eigen::Vector3d::Zero()).act(data.v[i]).isApprox(getVelocity(model,data,i,LOCAL_WORLD_ALIGNED)));
  }
}

BOOST_AUTO_TEST_CASE(test_get_acceleration)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  forwardKinematics(model,data,q,v,a);

  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    BOOST_CHECK(data.a[i].isApprox(getAcceleration(model,data,i)));
    BOOST_CHECK(data.a[i].isApprox(getAcceleration(model,data,i,LOCAL)));
    BOOST_CHECK(data.oMi[i].act(data.a[i]).isApprox(getAcceleration(model,data,i,WORLD)));
    BOOST_CHECK(SE3(data.oMi[i].rotation(), Eigen::Vector3d::Zero()).act(data.a[i]).isApprox(getAcceleration(model,data,i,LOCAL_WORLD_ALIGNED)));
  }
}

BOOST_AUTO_TEST_CASE(test_get_classical_acceleration)
{
  using namespace Eigen;
  using namespace pinocchio;

  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  VectorXd v(VectorXd::Random(model.nv));
  VectorXd a(VectorXd::Random(model.nv));

  forwardKinematics(model,data,q,v,a);

  for(Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
  {
    SE3 T = data.oMi[i];
    Motion vel = data.v[i];
    Motion acc = data.a[i];
    Vector3d linear;

    Motion acc_classical_local = acc;
    linear = acc.linear() + vel.angular().cross(vel.linear());
    acc_classical_local.linear() = linear;

    BOOST_CHECK(acc_classical_local.isApprox(getClassicalAcceleration(model,data,i)));
    BOOST_CHECK(acc_classical_local.isApprox(getClassicalAcceleration(model,data,i,LOCAL)));

    Motion vel_world = T.act(vel);
    Motion acc_classical_world = T.act(acc);
    linear = acc_classical_world.linear() + vel_world.angular().cross(vel_world.linear());
    acc_classical_world.linear() = linear;

    BOOST_CHECK(acc_classical_world.isApprox(getClassicalAcceleration(model,data,i,WORLD)));

    Motion vel_aligned = SE3(data.oMi[i].rotation(), Eigen::Vector3d::Zero()).act(vel);
    Motion acc_classical_aligned = SE3(data.oMi[i].rotation(), Eigen::Vector3d::Zero()).act(acc);
    linear = acc_classical_aligned.linear() + vel_aligned.angular().cross(vel_aligned.linear());
    acc_classical_aligned.linear() = linear;

    BOOST_CHECK(acc_classical_aligned.isApprox(getClassicalAcceleration(model,data,i,LOCAL_WORLD_ALIGNED)));
  }
}

BOOST_AUTO_TEST_CASE(test_kinematic_getters)
{
  using namespace Eigen;
  using namespace pinocchio;

  // Build a simple 2R planar model
  Model model;
  JointIndex jointId = 0;
  jointId = model.addJoint(jointId, JointModelRZ(), SE3::Identity(), "Joint1");
  jointId = model.addJoint(jointId, JointModelRZ(), SE3(Matrix3d::Identity(), Vector3d(1.0, 0.0, 0.0)), "Joint2");

  Data data(model);

  // Predetermined configuration values
  VectorXd q(model.nq);
  q << M_PI/2.0, 0.0;

  VectorXd v(model.nv);
  v << 1.0, 0.0;

  VectorXd a(model.nv);
  a << 0.0, 0.0;

  // Expected velocity
  Motion v_local;
  v_local.linear() = Vector3d(0.0, 1.0, 0.0);
  v_local.angular() = Vector3d(0.0, 0.0, 1.0);

  Motion v_world;
  v_world.linear() = Vector3d::Zero();
  v_world.angular() = Vector3d(0.0, 0.0, 1.0);

  Motion v_align;
  v_align.linear() = Vector3d(-1.0, 0.0, 0.0);
  v_align.angular() = Vector3d(0.0, 0.0, 1.0);

  // Expected classical acceleration
  Motion ac_local;
  ac_local.linear() = Vector3d(-1.0, 0.0, 0.0);
  ac_local.angular() = Vector3d::Zero();

  Motion ac_world = Motion::Zero();

  Motion ac_align;
  ac_align.linear() = Vector3d(0.0, -1.0, 0.0);
  ac_align.angular() = Vector3d::Zero();

  // Perform kinematics
  forwardKinematics(model,data,q,v,a);

  // Check output velocity
  BOOST_CHECK(v_local.isApprox(getVelocity(model,data,jointId)));
  BOOST_CHECK(v_local.isApprox(getVelocity(model,data,jointId,LOCAL)));
  BOOST_CHECK(v_world.isApprox(getVelocity(model,data,jointId,WORLD)));
  BOOST_CHECK(v_align.isApprox(getVelocity(model,data,jointId,LOCAL_WORLD_ALIGNED)));

  // Check output acceleration (all zero)
  BOOST_CHECK(getAcceleration(model,data,jointId).isZero());
  BOOST_CHECK(getAcceleration(model,data,jointId,LOCAL).isZero());
  BOOST_CHECK(getAcceleration(model,data,jointId,WORLD).isZero());
  BOOST_CHECK(getAcceleration(model,data,jointId,LOCAL_WORLD_ALIGNED).isZero());

  // Check output classical
  BOOST_CHECK(ac_local.isApprox(getClassicalAcceleration(model,data,jointId)));
  BOOST_CHECK(ac_local.isApprox(getClassicalAcceleration(model,data,jointId,LOCAL)));
  BOOST_CHECK(ac_world.isApprox(getClassicalAcceleration(model,data,jointId,WORLD)));
  BOOST_CHECK(ac_align.isApprox(getClassicalAcceleration(model,data,jointId,LOCAL_WORLD_ALIGNED)));
}

BOOST_AUTO_TEST_SUITE_END()
