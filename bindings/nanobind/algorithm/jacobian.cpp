// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/jacobian.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeJacobian(nb::module_ m)
{
  using Matrix6x = Data::Matrix6x;

  m.def(
    "computeJointJacobians",
    [](const Model & model, Data & data, ConstVectorRef q) -> const Matrix6x & {
      return computeJointJacobians(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Computes the full model Jacobian, i.e. the stack of all the motion subspaces expressed in "
    "the coordinate world frame.\n"
    "The result is accessible through data.J. This function computes also the forward "
    "kinematics of the model.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)");

  m.def(
    "computeJointJacobians",
    [](const Model & model, Data & data) -> const Matrix6x & {
      return computeJointJacobians(model, data);
    },
    "model"_a, "data"_a,
    "Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the "
    "world frame.\n"
    "The result is accessible through data.J. This function assumes that forward kinematics "
    "(pinocchio.forwardKinematics) has been called first.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model");

  m.def(
    "computeJointJacobian",
    [](const Model & model, Data & data, ConstVectorRef q, JointIndex joint_id) -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      computeJointJacobian(model, data, q, joint_id, J);
      return J;
    },
    "model"_a, "data"_a, "q"_a, "joint_id"_a,
    "Computes the Jacobian of a specific joint frame expressed in the local frame of the joint "
    "according to the given input configuration.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tjoint_id: index of the joint");

  m.def(
    "getJointJacobian",
    [](const Model & model, Data & data, JointIndex joint_id, ReferenceFrame rf) -> Matrix6x {
      Matrix6x J(Matrix6x::Zero(6, model.nv));
      getJointJacobian(model, data, joint_id, rf, J);
      return J;
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Computes the Jacobian of a given joint according to the given entries in data.\n"
    "If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local "
    "coordinate system of the joint.\n"
    "If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in "
    "the coordinate system of the frame centered on the joint, but aligned with the WORLD "
    "axes.\n"
    "If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate "
    "system of the frame associated to the WORLD.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");

  m.def(
    "computeJointJacobiansTimeVariation",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) -> const Matrix6x & {
      return computeJointJacobiansTimeVariation(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt "
    "which depends both on q and v. It also computes the joint Jacobian of the model (similar "
    "to computeJointJacobians). "
    "The result is accessible through data.dJ and data.J.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)");

  m.def(
    "getJointJacobianTimeVariation",
    [](const Model & model, Data & data, JointIndex joint_id, ReferenceFrame rf) -> Matrix6x {
      Matrix6x dJ(Matrix6x::Zero(6, model.nv));
      getJointJacobianTimeVariation(model, data, joint_id, rf, dJ);
      return dJ;
    },
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a,
    "Computes the Jacobian time variation of a specific joint expressed in the requested frame "
    "provided by the value of reference_frame. "
    "You have to call computeJointJacobiansTimeVariation first. This function also computes "
    "the full model Jacobian contained in data.J.\n"
    "If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local "
    "coordinate system of the joint.\n"
    "If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in "
    "the coordinate system of the frame centered on the joint, but aligned with the WORLD "
    "axes.\n"
    "If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate "
    "system of the frame associated to the WORLD.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint\n"
    "\treference_frame: reference frame in which the resulting derivatives are expressed");
}

PINOCCHIO_PYTHON_NAMESPACE_END
