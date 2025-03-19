//
// Copyright (c) 2015-2025 CNRS
//

#include "model-fixture.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <iostream>

#include <benchmark/benchmark.h>

static void CustomArguments(benchmark::internal::Benchmark * b)
{
  b->MinWarmUpTime(3.);
}

struct JacobianFixture : ModelFixture
{
  void SetUp(benchmark::State & st)
  {
    ModelFixture::SetUp(st);
    J.setZero(6, model.nv);
    joint_id = (pinocchio::Model::JointIndex)(model.njoints - 1);
    pinocchio::SE3 framePlacement = pinocchio::SE3::Random();
    frame_id = model.addFrame(
      pinocchio::Frame("test_frame", joint_id, 0, framePlacement, pinocchio::OP_FRAME));
    // Data must be recreated after model modification
    data = pinocchio::Data(model);
  }
  void TearDown(benchmark::State & st)
  {
    ModelFixture::TearDown(st);
  }

  pinocchio::Data::Matrix6x J;
  pinocchio::Model::JointIndex joint_id;
  pinocchio::Model::FrameIndex frame_id;
};

// FORWARD_KINEMATICS_Q

PINOCCHIO_DONT_INLINE static void forwardKinematicsQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::forwardKinematics(model, data, q);
}
BENCHMARK_DEFINE_F(JacobianFixture, FORWARD_KINEMATICS_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    forwardKinematicsQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, FORWARD_KINEMATICS_Q)->Apply(CustomArguments);

// COMPUTE_JOINT_JACOBIAN

PINOCCHIO_DONT_INLINE static void computeJointJacobianCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::Index joint_id,
  pinocchio::Data::Matrix6x & J)
{
  pinocchio::computeJointJacobian(model, data, q, joint_id, J);
}
BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_JOINT_JACOBIAN)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeJointJacobianCall(model, data, q, joint_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_JOINT_JACOBIAN)->Apply(CustomArguments);

// COMPUTE_JOINT_JACOBIANS_Q

PINOCCHIO_DONT_INLINE static void computeJointJacobiansQCall(
  const pinocchio::Model & model, pinocchio::Data & data, const Eigen::VectorXd & q)
{
  pinocchio::computeJointJacobians(model, data, q);
}
BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_JOINT_JACOBIANS_Q)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeJointJacobiansQCall(model, data, q);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_JOINT_JACOBIANS_Q)->Apply(CustomArguments);

// COMPUTE_JOINT_JACOBIANS

PINOCCHIO_DONT_INLINE static void
computeJointJacobiansCall(const pinocchio::Model & model, pinocchio::Data & data)
{
  pinocchio::computeJointJacobians(model, data);
}
BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_JOINT_JACOBIANS)(benchmark::State & st)
{
  pinocchio::forwardKinematics(model, data, q);
  for (auto _ : st)
  {
    computeJointJacobiansCall(model, data);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_JOINT_JACOBIANS)->Apply(CustomArguments);

// GET_JOINT_JACOBIAN

template<pinocchio::ReferenceFrame Frame>
PINOCCHIO_DONT_INLINE static void getJointJacobianCall(
  const pinocchio::Model & model,
  const pinocchio::Data & data,
  pinocchio::Index joint_id,
  const pinocchio::Data::Matrix6x & J)
{
  pinocchio::getJointJacobian(model, data, joint_id, Frame, J);
}

BENCHMARK_DEFINE_F(JacobianFixture, GET_JOINT_JACOBIAN_LOCAL)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  for (auto _ : st)
  {
    getJointJacobianCall<pinocchio::LOCAL>(model, data, joint_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_JOINT_JACOBIAN_LOCAL)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, GET_JOINT_JACOBIAN_WORLD)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  for (auto _ : st)
  {
    getJointJacobianCall<pinocchio::WORLD>(model, data, joint_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_JOINT_JACOBIAN_WORLD)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, GET_JOINT_JACOBIAN_LOCAL_WORLD_ALIGNED)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  for (auto _ : st)
  {
    getJointJacobianCall<pinocchio::LOCAL_WORLD_ALIGNED>(model, data, joint_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_JOINT_JACOBIAN_LOCAL_WORLD_ALIGNED)
  ->Apply(CustomArguments);

// GET_FRAME_JACOBIAN

template<pinocchio::ReferenceFrame Frame>
PINOCCHIO_DONT_INLINE static void getFrameJacobianCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  pinocchio::FrameIndex frame_id,
  const pinocchio::Data::Matrix6x & J)
{
  pinocchio::getFrameJacobian(model, data, frame_id, Frame, J);
}

BENCHMARK_DEFINE_F(JacobianFixture, GET_FRAME_JACOBIAN_LOCAL)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  std::cout << "frame_id: " << frame_id << std::endl;
  for (auto _ : st)
  {
    getFrameJacobianCall<pinocchio::LOCAL>(model, data, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_FRAME_JACOBIAN_LOCAL)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, GET_FRAME_JACOBIAN_WORLD)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  for (auto _ : st)
  {
    getFrameJacobianCall<pinocchio::WORLD>(model, data, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_FRAME_JACOBIAN_WORLD)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, GET_FRAME_JACOBIAN_LOCAL_WORLD_ALIGNED)(benchmark::State & st)
{
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  for (auto _ : st)
  {
    getFrameJacobianCall<pinocchio::LOCAL_WORLD_ALIGNED>(model, data, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, GET_FRAME_JACOBIAN_LOCAL_WORLD_ALIGNED)
  ->Apply(CustomArguments);

// COMPUTE_FRAME_JACOBIAN

template<pinocchio::ReferenceFrame Frame>
PINOCCHIO_DONT_INLINE static void computeFrameJacobianCall(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q,
  pinocchio::FrameIndex frame_id,
  const pinocchio::Data::Matrix6x & J)
{
  pinocchio::computeFrameJacobian(model, data, q, frame_id, Frame, J);
}

BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_LOCAL)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeFrameJacobianCall<pinocchio::LOCAL>(model, data, q, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_LOCAL)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_WORLD)(benchmark::State & st)
{
  for (auto _ : st)
  {
    computeFrameJacobianCall<pinocchio::WORLD>(model, data, q, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_WORLD)->Apply(CustomArguments);
BENCHMARK_DEFINE_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_LOCAL_WORLD_ALIGNED)(
  benchmark::State & st)
{
  for (auto _ : st)
  {
    computeFrameJacobianCall<pinocchio::LOCAL_WORLD_ALIGNED>(model, data, q, frame_id, J);
  }
}
BENCHMARK_REGISTER_F(JacobianFixture, COMPUTE_FRAME_JACOBIAN_LOCAL_WORLD_ALIGNED)
  ->Apply(CustomArguments);

PINOCCHIO_BENCHMARK_MAIN();
