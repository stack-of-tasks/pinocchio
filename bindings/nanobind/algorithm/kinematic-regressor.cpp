// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/regressor.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeKinematicRegressor(nb::module_ m)
{
  using Matrix6x = Data::Matrix6x;

  m.def(
    "computeJointKinematicRegressor",
    [](
      const Model & model, const Data & data, JointIndex joint_id, ReferenceFrame rf,
      const SE3 & placement) -> Matrix6x {
      return computeJointKinematicRegressor(model, data, joint_id, rf, placement);
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a, "placement"_a,
    "Computes the kinematic regressor that links the joint placements variations of the whole "
    "kinematic tree to the placement variation of the frame rigidly attached to the joint and "
    "given by its placement w.r.t. to the joint frame.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the result is expressed (LOCAL, "
    "LOCAL_WORLD_ALIGNED or WORLD)\n"
    "\tplacement: relative placement to the joint frame");

  m.def(
    "computeJointKinematicRegressor",
    [](const Model & model, const Data & data, JointIndex joint_id, ReferenceFrame rf) -> Matrix6x {
      return computeJointKinematicRegressor(model, data, joint_id, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Computes the kinematic regressor that links the joint placement variations of the whole "
    "kinematic tree to the placement variation of the joint given as input.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the result is expressed (LOCAL, "
    "LOCAL_WORLD_ALIGNED or WORLD)");

  m.def(
    "computeFrameKinematicRegressor",
    [](const Model & model, Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Matrix6x {
      return computeFrameKinematicRegressor(model, data, frame_id, rf);
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the kinematic regressor that links the joint placement variations of the whole "
    "kinematic tree to the placement variation of the frame given as input.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tframe_id: index of the frame\n"
    "\treference_frame: reference frame in which the result is expressed (LOCAL, "
    "LOCAL_WORLD_ALIGNED or WORLD)");
}

PINOCCHIO_PYTHON_NAMESPACE_END
