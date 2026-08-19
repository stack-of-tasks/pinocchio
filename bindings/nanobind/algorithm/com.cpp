// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

using Vector3 = Eigen::Matrix<Scalar, 3, 1, Options>;
using Matrix3x = Eigen::Matrix<Scalar, 3, Eigen::Dynamic, Options>;

void exposeCOM(nb::module_ m)
{
  m.def(
    "computeTotalMass",
    [](const Model & model) -> Scalar { return pinocchio::computeTotalMass(model); }, "model"_a,
    "Compute the total mass of the model and return it.");

  m.def(
    "computeTotalMass",
    [](const Model & model, Data & data) -> Scalar {
      return pinocchio::computeTotalMass(model, data);
    },
    "model"_a, "data"_a,
    "Compute the total mass of the model, put it in data.mass[0] and return it.");

  m.def(
    "computeSubtreeMasses", &computeSubtreeMasses<Scalar, Options, JointCollectionDefaultTpl>,
    "model"_a, "data"_a,
    "Compute the mass of each kinematic subtree and store it in the vector data.mass.");

  m.def(
    "centerOfMass",
    [](const Model & model, Data & data, ConstVectorRef q, bool compute_subtree_coms) -> Vector3 {
      return pinocchio::centerOfMass(model, data, q, compute_subtree_coms);
    },
    "model"_a, "data"_a, "q"_a, "compute_subtree_coms"_a = true,
    "Compute the center of mass, putting the result in data and return it.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
    "subtrees.");

  m.def(
    "centerOfMass",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v,
      bool compute_subtree_coms) -> Vector3 {
      return pinocchio::centerOfMass(model, data, q, v, compute_subtree_coms);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "compute_subtree_coms"_a = true,
    "Computes the center of mass position and velocity by storing the result in data.\n"
    "It returns the center of mass position expressed in the WORLD frame.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
    "subtrees.");

  m.def(
    "centerOfMass",
    [](
      const Model & model, Data & data, ConstVectorRef q, ConstVectorRef v, ConstVectorRef a,
      bool compute_subtree_coms) -> Vector3 {
      return pinocchio::centerOfMass(model, data, q, v, a, compute_subtree_coms);
    },
    "model"_a, "data"_a, "q"_a, "v"_a, "a"_a, "compute_subtree_coms"_a = true,
    "Computes the center of mass position, velocity and acceleration by storing the result in "
    "data. It returns the center of mass position expressed in the WORLD frame.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
    "subtrees.");

  m.def(
    "centerOfMass",
    [](
      const Model & model, Data & data, pinocchio::KinematicLevel kinematic_level,
      bool compute_subtree_coms) -> Data::Vector3 {
      return pinocchio::centerOfMass(model, data, kinematic_level, compute_subtree_coms);
    },
    "model"_a, "data"_a, "kinematic_level"_a, "compute_subtree_coms"_a = true,
    "Computes the center of mass position, velocity or acceleration of a given model according "
    "to the current kinematic values contained in data and the requested kinematic_level.\n"
    "If kinematic_level = POSITION, computes the CoM position, if kinematic_level = VELOCITY, "
    "also computes the CoM velocity and if kinematic_level = ACCELERATION, it also computes "
    "the CoM acceleration.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
    "subtrees.");

  m.def(
    "centerOfMass",
    [](const Model & model, Data & data, bool compute_subtree_coms) -> Data::Vector3 {
      return pinocchio::centerOfMass(model, data, compute_subtree_coms);
    },
    "model"_a, "data"_a, "compute_subtree_coms"_a = true,
    "Computes the center of mass position, velocity and acceleration of a given model "
    "according to the current kinematic values contained in data.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of "
    "the subtrees.");

  m.def(
    "jacobianCenterOfMass",
    [](const Model & model, Data & data, ConstVectorRef q, bool compute_subtree_coms)
      -> Data::Matrix3x {
      return pinocchio::jacobianCenterOfMass(model, data, q, compute_subtree_coms);
    },
    "model"_a, "data"_a, "q"_a, "compute_subtree_coms"_a = true,
    "Computes the Jacobian of the center of mass, puts the result in data and return it.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
    "subtrees.");

  m.def(
    "jacobianCenterOfMass",
    [](const Model & model, Data & data, bool compute_subtree_coms) -> Data::Matrix3x {
      return pinocchio::jacobianCenterOfMass(model, data, compute_subtree_coms);
    },
    "model"_a, "data"_a, "compute_subtree_coms"_a = true,
    "Computes the Jacobian of the center of mass, puts the result in data and return it.\n"
    "If compute_subtree_coms is True, the algorithm also computes the center of mass of "
    "the subtrees.");

  m.def(
    "jacobianSubtreeCenterOfMass",
    [](const Model & model, Data & data, ConstVectorRef q, pinocchio::JointIndex joint_id)
      -> Matrix3x {
      Matrix3x J = Matrix3x::Zero(3, model.nv);
      pinocchio::jacobianSubtreeCenterOfMass(model, data, q, joint_id, J);
      return J;
    },
    "model"_a, "data"_a, "q"_a, "subtree_root_joint_id"_a,
    "Computes the Jacobian of the CoM of the given subtree expressed in the WORLD frame, "
    "according to the given joint configuration.");

  m.def(
    "jacobianSubtreeCenterOfMass",
    [](const Model & model, Data & data, pinocchio::JointIndex joint_id) -> Matrix3x {
      Matrix3x J = Matrix3x::Zero(3, model.nv);
      pinocchio::jacobianSubtreeCenterOfMass(model, data, joint_id, J);
      return J;
    },
    "model"_a, "data"_a, "subtree_root_joint_id"_a,
    "Computes the Jacobian of the CoM of the given subtree expressed in the WORLD frame, "
    "according to the given entries in data.");

  m.def(
    "getJacobianSubtreeCenterOfMass",
    [](const Model & model, Data & data, pinocchio::JointIndex joint_id) -> Matrix3x {
      Matrix3x J = Matrix3x::Zero(3, model.nv);
      pinocchio::getJacobianSubtreeCenterOfMass(model, data, joint_id, J);
      return J;
    },
    "model"_a, "data"_a, "subtree_root_joint_id"_a,
    "Get the Jacobian of the CoM of the given subtree expressed in the world frame, "
    "according to the given entries in data. It assumes that jacobianCenterOfMass has "
    "been called first.");
}
PINOCCHIO_PYTHON_NAMESPACE_END
