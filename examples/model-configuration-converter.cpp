#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph-algo.hpp"
#include "pinocchio/parsers/graph/model-configuration-converter.hpp"

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <iostream>

/// This example show how to:
///  - Construct a kinematics chain with ModelGraph API
///  - Build a Model with b1 as root body with a fixed base
///  - Build a Model with b3 as root body with a free flyer as root joint
///  - Use ModelConfigurationConverter API to convert configuration and velocity vector from the
///    first model to the second
int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  /// Construct kinematics chain with ModelGraph API.
  graph::ModelGraph g;
  Inertia I_I(Inertia::Identity());
  SE3 X_I(SE3::Identity());

  g.addBody("b1", I_I);
  g.addBody("b2", I_I);
  g.addBody("b3", I_I);
  g.addJoint(
    "j1", graph::JointRevolute(Eigen::Vector3d::UnitZ()), "b1", SE3::Random(), "b2", SE3::Random());
  g.addJoint(
    "j2", graph::JointPrismatic(Eigen::Vector3d::UnitX()), "b2", SE3::Random(), "b3",
    SE3::Random());

  /// Create a Model with b1 as root body and a fixed base.
  const auto forward_build = graph::buildModelWithBuildInfo(g, "b1", X_I);
  const auto & forward_model = forward_build.model;
  Data forward_data(forward_model);

  /// Compute b3 placement and velocity with forward model.
  const Eigen::VectorXd qmax = Eigen::VectorXd::Ones(forward_model.nq);
  const Eigen::VectorXd forward_q = randomConfiguration(forward_model, -qmax, qmax);
  const Eigen::VectorXd forward_v(Eigen::VectorXd::Random(forward_model.nv));
  forwardKinematics(forward_model, forward_data, forward_q, forward_v);
  updateFramePlacements(forward_model, forward_data);

  auto b3_index = forward_model.getFrameId("b3", BODY);
  auto X_b3 = forward_data.oMf[b3_index];
  auto V_b3 = getFrameVelocity(forward_model, forward_data, b3_index);

  /// Create the backward model with b3 as root body and a free flyer as root joint.
  /// b3 is placed at the same position and same velocity than b3 in the forward model.
  const auto backward_build =
    graph::buildModelWithBuildInfo(g, "b3", X_b3, graph::JointFreeFlyer());
  const auto & backward_model = backward_build.model;
  Data backward_data(backward_model);
  Eigen::VectorXd backward_q = neutral(backward_model);
  Eigen::VectorXd backward_v(Eigen::VectorXd::Zero(backward_model.nv));
  backward_v.segment<6>(0) = V_b3.toVector();

  /// Create the converter and convert the configuration and velocity vector.
  auto f_to_b_converter = graph::createConverter(
    forward_model, backward_model, forward_build.build_info, backward_build.build_info);
  f_to_b_converter.convertConfigurationVector(forward_q, backward_q);
  f_to_b_converter.convertTangentVector(forward_q, forward_v, backward_v);
  forwardKinematics(backward_model, backward_data, backward_q, backward_v);
  updateFramePlacements(backward_model, backward_data);

  /// Show that frame configuration and velocities are the same.
  for (std::size_t i = 0; i < forward_model.frames.size(); ++i)
  {
    const auto & frame = forward_model.frames[i];
    if (frame.type == FrameType::BODY)
    {
      auto i_b = backward_model.getFrameId(frame.name, frame.type);
      auto motion_f = getFrameVelocity(forward_model, forward_data, i);
      auto motion_b = getFrameVelocity(backward_model, backward_data, i_b);

      std::cout << "Frame " << frame.name << " configuration and velocity in forward_model:\n";
      std::cout << forward_data.oMf[i] << motion_f << std::endl;
      std::cout << "Frame " << frame.name << " configuration and velocity in backward_model:\n";
      std::cout << backward_data.oMf[i_b] << motion_b << std::endl << std::endl;
    }
  }
}
