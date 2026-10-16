// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/eigen/tensor.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeKinematicsDerivatives(nb::module_ m)
{
  using Matrix6x = Data::Matrix6x;
  using Matrix3x = Data::Matrix3x;

  m.def(
    "computeForwardKinematicsDerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) {
      computeForwardKinematicsDerivatives(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Computes all the terms required to compute the derivatives of the placement and spatial "
    "velocities\n"
    "for any joint/frame of the model.\n"
    "The results are stored in data.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n");

  m.def(
    "computeForwardKinematicsDerivatives",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a) {
      computeForwardKinematicsDerivatives(model, data, q, v, a);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a,
    "Computes all the terms required to compute the derivatives of the placement, spatial "
    "velocities and accelerations\n"
    "for any joint/frame of the model.\n"
    "The results are stored in data.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)\n");

  m.def(
    "getJointVelocityDerivatives",
    [](const Model & model, Data & data, JointIndex joint_id, ReferenceFrame rf) -> nb::tuple {
      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));
      getJointVelocityDerivatives(model, data, joint_id, rf, partial_dq, partial_dv);
      return nb::make_tuple(std::move(partial_dq), std::move(partial_dv));
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial velocity of a given joint with respect to\n"
    "the joint configuration and velocity and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

  m.def(
    "getPointVelocityDerivatives",
    [](
      const Model & model, Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> nb::tuple {
      Matrix3x v_partial_dq(Matrix3x::Zero(3, model.nv));
      Matrix3x v_partial_dv(Matrix3x::Zero(3, model.nv));
      getPointVelocityDerivatives(model, data, joint_id, placement, rf, v_partial_dq, v_partial_dv);
      return nb::make_tuple(std::move(v_partial_dq), std::move(v_partial_dv));
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a,
    "Computes the partial derivatives of the velocity of a point given by its placement "
    "information w.r.t. the joint frame and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\tplacement: relative placement of the point w.r.t. the joint frame\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

  m.def(
    "getJointAccelerationDerivatives",
    [](const Model & model, Data & data, JointIndex joint_id, ReferenceFrame rf) -> nb::tuple {
      Matrix6x v_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6, model.nv));
      getJointAccelerationDerivatives(
        model, data, joint_id, rf, v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);
      return nb::make_tuple(
        std::move(v_partial_dq), std::move(a_partial_dq), std::move(a_partial_dv),
        std::move(a_partial_da));
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Computes the partial derivatives of the spatial acceleration of a given joint with "
    "respect to\n"
    "the joint configuration, velocity and acceleration and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

  m.def(
    "getPointClassicAccelerationDerivatives",
    [](
      const Model & model, Data & data, JointIndex joint_id, const SE3 & placement,
      ReferenceFrame rf) -> nb::tuple {
      Matrix3x v_partial_dq(Matrix3x::Zero(3, model.nv));
      Matrix3x a_partial_dq(Matrix3x::Zero(3, model.nv));
      Matrix3x a_partial_dv(Matrix3x::Zero(3, model.nv));
      Matrix3x a_partial_da(Matrix3x::Zero(3, model.nv));
      getPointClassicAccelerationDerivatives(
        model, data, joint_id, placement, rf, v_partial_dq, a_partial_dq, a_partial_dv,
        a_partial_da);
      return nb::make_tuple(
        std::move(v_partial_dq), std::move(a_partial_dq), std::move(a_partial_dv),
        std::move(a_partial_da));
    },
    "model"_a, "data"_a, "joint_id"_a, "placement"_a, "reference_frame"_a,
    "Computes the partial derivatives of the classic acceleration of a point given by its "
    "placement information w.r.t. the joint frame and returns them as a tuple.\n"
    "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
    "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
    "reference_frame.\n"
    "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\tplacement: relative placement of the point w.r.t. the joint frame\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed\n");

  m.def(
    "getCenterOfMassVelocityDerivatives",
    [](const Model & model, Data & data) -> Matrix3x {
      Matrix3x partial_dq(Matrix3x::Zero(3, model.nv));
      getCenterOfMassVelocityDerivatives(model, data, partial_dq);
      return partial_dq;
    },
    "model"_a, "data"_a,
    "Computes the partial derivaties of the center of mass velocity with respect to\n"
    "the joint configuration.\n"
    "You must first call computeAllTerms(model,data,q,v) or centerOfMass(model,data,q,v) "
    "before calling this function.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n");

  m.def(
    "computeJointKinematicHessians",
    [](const Model & model, Data & data) { computeJointKinematicHessians(model, data); }, "model"_a,
    "data"_a,
    "Computes all the terms required to compute the second order derivatives of the placement "
    "information, also know as the kinematic Hessian. This function assumes that the joint "
    "Jacobians (a.k.a data.J) has been computed first. See also computeJointJacobians for such "
    "a function.");

  m.def(
    "computeJointKinematicHessians",
    [](const Model & model, Data & data, ConstVectorRef q) {
      computeJointKinematicHessians(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Computes all the terms required to compute the second order derivatives of the placement "
    "information, also know as the kinematic Hessian.");

  using Tensor3 = Eigen::Tensor<Scalar, 3, Options>;

  m.def(
    "getJointKinematicHessian",
    [](const Model & model, const Data & data, JointIndex joint_id, ReferenceFrame rf) -> Tensor3 {
      return getJointKinematicHessian(model, data, joint_id, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Retrieves the kinematic Hessian of a given joint according to the values already computed "
    "by computeJointKinematicHessians and stored in data.\n"
    "While the kinematic Jacobian of a given joint frame corresponds to the first order "
    "derivative of the placement variation with respect to q, the kinematic Hessian "
    "corresponds to the second order derivation of placement variation, which in turns also "
    "corresponds to the first order derivative of the kinematic Jacobian.");

  m.def(
    "getFrameKinematicHessian",
    [](const Model & model, const Data & data, FrameIndex frame_id, ReferenceFrame rf) -> Tensor3 {
      return getFrameKinematicHessian(model, data, frame_id, rf);
    },
    "model"_a, "data"_a, "frame_id"_a, "reference_frame"_a,
    "Retrieves the kinematic Hessian of a given frame according to the values already computed "
    "by computeJointKinematicHessians and stored in data.\n"
    "While the kinematic Jacobian of a given joint frame corresponds to the first order "
    "derivative of the placement variation with respect to q, the kinematic Hessian "
    "corresponds to the second order derivation of placement variation, which in turns also "
    "corresponds to the first order derivative of the kinematic Jacobian.");

  m.def(
    "getFrameKinematicHessian",
    [](
      const Model & model, const Data & data, JointIndex joint_id, const SE3 & frame_placement,
      ReferenceFrame rf) -> Tensor3 {
      return getFrameKinematicHessian(model, data, joint_id, frame_placement, rf);
    },
    "model"_a, "data"_a, "joint_id"_a, "frame_placement"_a, "reference_frame"_a,
    "Retrieves the kinematic Hessian of a given frame, given by the related joint_id and "
    "frame_placement, leveraging to the values already computed "
    "by computeJointKinematicHessians and stored in data.\n"
    "While the kinematic Jacobian of a given joint frame corresponds to the first order "
    "derivative of the placement variation with respect to q, the kinematic Hessian "
    "corresponds to the second order derivation of placement variation, which in turns also "
    "corresponds to the first order derivative of the kinematic Jacobian.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
