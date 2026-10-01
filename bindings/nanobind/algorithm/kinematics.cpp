// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/kinematics.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeKinematics(nb::module_ m)
{
  m.def(
    "updateGlobalPlacements", &updateGlobalPlacements<Scalar, Options, JointCollectionDefaultTpl>,
    "model"_a, "data"_a,
    "Updates the global placements of all joint frames of the kinematic "
    "tree and store the results in data according to the relative placements of the joints.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model");

  m.def(
    "getVelocity", &getVelocity<Scalar, Options, JointCollectionDefaultTpl>, //*
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial velocity of the joint expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial "
    "velocity stored in `data.v`");

  m.def(
    "getAcceleration", &getAcceleration<Scalar, Options, JointCollectionDefaultTpl>, //
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a = LOCAL,
    "Returns the spatial acceleration of the joint expressed in the coordinate system given by "
    "reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in `data.a`.");

  m.def(
    "getClassicalAcceleration",
    &getClassicalAcceleration<Scalar, Options, JointCollectionDefaultTpl>, //
    "model"_a, "data"_a, "joint_id"_a, "reference_frame"_a = LOCAL,
    "Returns the \"classical\" acceleration of the joint expressed in the coordinate system "
    "given by reference_frame.\n"
    "forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial "
    "acceleration stored in data.a .");

  m.def(
    "forwardKinematics",
    [](const Model & model, Data & data, ConstVectorRef q) { forwardKinematics(model, data, q); },
    "model"_a, "data"_a, "q"_a,
    "Compute the global placements of all the joints of the kinematic "
    "tree and store the results in data.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)");

  m.def(
    "forwardKinematics",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v) {
      forwardKinematics(model, data, q, v);
    },
    "model"_a, "data"_a, "q"_a, "v"_a,
    "Compute the global placements and local spatial velocities of all the joints of the "
    "kinematic tree and store the results in data.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)");

  m.def(
    "forwardKinematics",
    [](const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a) {
      forwardKinematics(model, data, q, v, a);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a,
    "Compute the global placements, local spatial velocities and spatial accelerations of all "
    "the joints of the kinematic tree and store the results in data.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree\n"
    "\tdata: data related to the model\n"
    "\tq: the joint configuration vector (size model.nq)\n"
    "\tv: the joint velocity vector (size model.nv)\n"
    "\ta: the joint acceleration vector (size model.nv)");
}

PINOCCHIO_PYTHON_NAMESPACE_END
