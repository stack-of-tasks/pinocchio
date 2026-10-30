// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeFramesAlgo(nb::module_ m)
{
  using Matrix6x = Data::Matrix6x;

  m.def(
    "updateFramePlacements",
    [](const Model & model, Data & data) { updateFramePlacements(model, data); }, "model"_a,
    "data"_a,
    "Computes the placements of all the operational frames according to the current joint "
    "placement stored in data and puts the results in data.");

  m.def(
    "updateFramePlacement",
    [](const Model & model, Data & data, FrameIndex frame_id) -> const SE3 & {
      return updateFramePlacement(model, data, frame_id);
    },
    "model"_a, "data"_a, "frame_id"_a,
    "Computes the placement of the given operational frame (frame_id) according to the current "
    "joint placement stored in data, stores the results in data and returns it.");

  m.def(
    "framesForwardKinematics",
    [](const Model & model, Data & data, ConstVectorRef q) {
      framesForwardKinematics(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Calls first the forwardKinematics(model,data,q) and then update the Frame placement "
    "quantities (data.oMf).");

  m.def(
    "getFrameVelocity",
    [](const Model & model, const Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Motion {
      return getFrameVelocity(model, data, frame_id, rf);
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial velocity of the frame expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial "
    "velocity stored in data.v");

  m.def(
    "getFrameVelocity",
    [](
      const Model & model, const Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> Motion {
      return getFrameVelocity(model, data, joint_id, placement, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial velocity of the frame expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial "
    "velocity stored in data.v");

  m.def(
    "getFrameAcceleration",
    [](const Model & model, const Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Motion {
      return getFrameAcceleration(model, data, frame_id, rf);
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial acceleration of the frame expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in data.a.");

  m.def(
    "getFrameAcceleration",
    [](
      const Model & model, const Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> Motion {
      return getFrameAcceleration(model, data, joint_id, placement, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial acceleration of the frame expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in data.a.");

  m.def(
    "getFrameClassicalAcceleration",
    [](const Model & model, const Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Motion {
      return getFrameClassicalAcceleration(model, data, frame_id, rf);
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a = LOCAL,
    "Returns the \"classical\" acceleration of the frame expressed in the coordinate system "
    "given by reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in data.a.");

  m.def(
    "getFrameClassicalAcceleration",
    [](
      const Model & model, const Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> Motion {
      return getFrameClassicalAcceleration(model, data, joint_id, placement, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a = LOCAL,
    "Returns the \"classical\" acceleration of the frame expressed in the coordinate system "
    "given by reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in data.a.");

  m.def(
    "computeFrameJacobian",
    [](const Model & model, Data & data, ConstVectorRef q, FrameIndex frame_id, ReferenceFrame rf)
      -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      computeFrameJacobian(model, data, q, frame_id, rf, J);
      return J;
    },
    "model"_a, "data"_a, "q"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the Jacobian of the frame given by its frame_id in the coordinate system given "
    "by reference_frame.");

  m.def(
    "computeFrameJacobian",
    [](const Model & model, Data & data, ConstVectorRef q, FrameIndex frame_id) -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      computeFrameJacobian(model, data, q, frame_id, J);
      return J;
    },
    "model"_a, "data"_a, "q"_a, "frame_id"_a,
    "Computes the Jacobian of the frame given by its frame_id.\n"
    "The columns of the Jacobian are expressed in the coordinates system of the Frame itself.\n"
    "In other words, the velocity of the frame vF expressed in the local coordinate is given "
    "by J*v, where v is the joint velocity.");

  m.def(
    "getFrameJacobian",
    [](const Model & model, Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      getFrameJacobian(model, data, frame_id, rf, J);
      return J;
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the Jacobian of the frame given by its ID either in the LOCAL, "
    "LOCAL_WORLD_ALIGNED or the WORLD coordinates systems.\n"
    "In other words, the velocity of the frame vF expressed in the reference frame is given by "
    "J*v, where v is the joint velocity vector.\n"
    "remarks: computeJointJacobians(model,data,q) must have been called first.");

  m.def(
    "getFrameJacobian",
    [](
      const Model & model, Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      getFrameJacobian(model, data, joint_id, placement, rf, J);
      return J;
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a,
    "Computes the Jacobian of the frame given by its placement with respect to the Joint frame "
    "and expressed the solution either in the LOCAL, LOCAL_WORLD_ALIGNED or the WORLD "
    "coordinates systems.\n"
    "In other words, the velocity of the frame vF expressed in the reference frame is given by "
    "J*v, where v is the joint velocity vector.\n"
    "remarks: computeJointJacobians(model,data,q) must have been called first.");

  m.def(
    "getFrameJacobianTimeVariation",
    [](const Model & model, Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Matrix6x {
      Matrix6x dJ(Matrix6x::Zero(6, model.nv));
      getFrameJacobianTimeVariation(model, data, frame_id, rf, dJ);
      return dJ;
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Returns the Jacobian time variation of the frame given by its frame_id either in the "
    "reference frame provided by reference_frame.\n"
    "You have to call computeJointJacobiansTimeVariation(model,data,q,v) and "
    "updateFramePlacements(model,data) first.");

  m.def(
    "frameJacobianTimeVariation",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, FrameIndex frame_id,
      ReferenceFrame rf) -> Matrix6x {
      computeJointJacobiansTimeVariation(model, data, q, v);
      updateFramePlacements(model, data);
      Matrix6x dJ(Matrix6x::Zero(6, model.nv));
      getFrameJacobianTimeVariation(model, data, frame_id, rf, dJ);
      return dJ;
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "frame_id"_a, "reference_frame"_a,
    "Computes the Jacobian time variation of the frame given by its frame_id in the "
    "reference frame provided by reference_frame.");

  m.def(
    "computeSupportedInertiaByFrame",
    [](const Model & model, const Data & data, FrameIndex frame_id, bool with_subtree) -> Inertia {
      return computeSupportedInertiaByFrame(model, data, frame_id, with_subtree);
    },
    "model"_a, "data"_a, "frame_id"_a, "with_subtree"_a,
    "Computes the supported inertia by the frame (given by frame_id) and returns it.\n"
    "The supported inertia corresponds to the sum of the inertias of all the child frames "
    "(that belongs to the same joint body) and the child joints, if with_subtree=True.\n"
    "You must first call pinocchio::forwardKinematics to update placement values in data "
    "structure.");

  m.def(
    "computeSupportedForceByFrame",
    [](const Model & model, const Data & data, FrameIndex frame_id) -> Force {
      return computeSupportedForceByFrame(model, data, frame_id);
    },
    "model"_a, "data"_a, "frame_id"_a,
    "Computes the supported force of the frame (given by frame_id) and returns it.\n"
    "The supported force corresponds to the sum of all the forces experienced after the given "
    "frame.\n"
    "You must first call pinocchio::rnea to update placement values in data structure.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
