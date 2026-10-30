// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/regressor.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeRegressor(nb::module_ m)
{
  using Matrix3x = Data::Matrix3x;
  using BodyRegressorType = Data::BodyRegressorType;
  using MatrixXs = Data::MatrixXs;
  using RowVectorXs = Data::RowVectorXs;

  m.def(
    "computeStaticRegressor",
    [](const Model & model, Data & data, ConstVectorRef q) -> const Matrix3x & {
      return computeStaticRegressor(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Compute the static regressor that links the inertia parameters of the system to its "
    "center of mass position,\n"
    "store the result in data and return it.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)");

  m.def(
    "bodyRegressor",
    [](const Motion & v, const Motion & a) -> BodyRegressorType { return bodyRegressor(v, a); },
    "velocity"_a, "acceleration"_a,
    "Computes the regressor for the dynamic parameters of a single rigid body.\n"
    "The result is such that "
    "Ia + v x Iv = bodyRegressor(v,a) * I.toDynamicParameters()\n\n"
    "Parameters:\n"
    "\tvelocity: spatial velocity of the rigid body\n"
    "\tacceleration: spatial acceleration of the rigid body");

  m.def(
    "jointBodyRegressor",
    [](const Model & model, Data & data, JointIndex joint_id) -> const BodyRegressorType & {
      return jointBodyRegressor(model, data, joint_id);
    },
    "model"_a, "data"_a, "joint_id"_a,
    "Compute the regressor for the dynamic parameters of a rigid body attached to a given "
    "joint.\n"
    "This algorithm assumes RNEA has been run to compute the acceleration and gravitational "
    "effects.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tjoint_id: index of the joint");

  m.def(
    "frameBodyRegressor",
    [](const Model & model, Data & data, FrameIndex frame_id) -> const BodyRegressorType & {
      return frameBodyRegressor(model, data, frame_id);
    },
    "model"_a, "data"_a, "frame_id"_a,
    "Computes the regressor for the dynamic parameters of a rigid body attached to a given "
    "frame.\n"
    "This algorithm assumes RNEA has been run to compute the acceleration and gravitational "
    "effects.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tframe_id: index of the frame");

  m.def(
    "computeJointTorqueRegressor",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a)
      -> const MatrixXs & { return computeJointTorqueRegressor(model, data, q, v, a); },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a,
    "Compute the joint torque regressor that links the joint torque "
    "to the dynamic parameters of each link according to the current the robot motion,\n"
    "store the result in data and return it.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)");

  m.def(
    "computeKineticEnergyRegressor",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v)
      -> const RowVectorXs & { return computeKineticEnergyRegressor(model, data, q, v); },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Compute the kinetic energy regressor that links the kinetic energy "
    "to the dynamic parameters of each link according to the current the robot motion,\n"
    "store the result in data and return it.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)");

  m.def(
    "computePotentialEnergyRegressor",
    [](const Model & model, Data & data, ConstVectorRef q) -> const RowVectorXs & {
      return computePotentialEnergyRegressor(model, data, q);
    },
    "model"_a, "data"_a, "q"_a,
    "Compute the potential energy regressor that links the potential energy "
    "to the dynamic parameters of each link according to the current the robot motion,\n"
    "store the result in data and return it.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)");
}

PINOCCHIO_PYTHON_NAMESPACE_END
