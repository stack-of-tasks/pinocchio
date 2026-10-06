// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/frames-derivatives.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeFramesDerivatives(nb::module_ m)
{
  using Matrix6x = Data::Matrix6x;

  m.def(
    "getFrameVelocityDerivatives",
    [](const Model & model, Data & data, FrameIndex frame_id, ReferenceFrame rf) -> nb::tuple {
      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));
      getFrameVelocityDerivatives(model, data, frame_id, rf, partial_dq, partial_dv);
      return nb::make_tuple(std::move(partial_dq), std::move(partial_dv));
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial velocity of a given frame with respect to\n"
    "the joint configuration and velocity and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tframe_id: index of the frame\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");

  m.def(
    "getFrameVelocityDerivatives",
    [](
      const Model & model, Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> nb::tuple {
      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));
      getFrameVelocityDerivatives(model, data, joint_id, placement, rf, partial_dq, partial_dv);
      return nb::make_tuple(std::move(partial_dq), std::move(partial_dv));
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial velocity of a frame given by its relative "
    "placement, with respect to\n"
    "the joint configuration and velocity and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\tplacement: placement of the Frame w.r.t. the joint frame.\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");

  m.def(
    "getFrameAccelerationDerivatives",
    [](const Model & model, Data & data, FrameIndex frame_id, ReferenceFrame rf) -> nb::tuple {
      Matrix6x v_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6, model.nv));
      getFrameAccelerationDerivatives(
        model, data, frame_id, rf, v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);
      return nb::make_tuple(
        std::move(v_partial_dq), std::move(a_partial_dq), std::move(a_partial_dv),
        std::move(a_partial_da));
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial acceleration of a given frame with "
    "respect to\n"
    "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tframe_id: index of the frame\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");

  m.def(
    "getFrameAccelerationDerivatives",
    [](
      const Model & model, Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> nb::tuple {
      Matrix6x v_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6, model.nv));
      getFrameAccelerationDerivatives(
        model, data, joint_id, placement, rf, v_partial_dq, a_partial_dq, a_partial_dv,
        a_partial_da);
      return nb::make_tuple(
        std::move(v_partial_dq), std::move(a_partial_dq), std::move(a_partial_dv),
        std::move(a_partial_da));
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial acceleration of a frame given by its "
    "relative placement, with respect to\n"
    "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\tplacement: placement of the Frame w.r.t. the joint frame.\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");
}

PINOCCHIO_PYTHON_NAMESPACE_END
