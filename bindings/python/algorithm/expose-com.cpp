//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <boost/python/overloads.hpp>

namespace pinocchio
{
  namespace python
  {

    static context::SE3::Vector3 com_0_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      bool computeSubtreeComs = true)
    {
      return centerOfMass(model, data, q, computeSubtreeComs);
    }

    static context::SE3::Vector3 com_1_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      bool computeSubtreeComs = true)
    {
      return centerOfMass(model, data, q, v, computeSubtreeComs);
    }

    static context::SE3::Vector3 com_2_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & a,
      bool computeSubtreeComs = true)
    {
      return centerOfMass(model, data, q, v, a, computeSubtreeComs);
    }

    static const context::Data::Vector3 & com_level_proxy(
      const context::Model & model,
      context::Data & data,
      KinematicLevel kinematic_level,
      bool computeSubtreeComs = true)
    {
      return centerOfMass(model, data, kinematic_level, computeSubtreeComs);
    }

    static const context::Data::Vector3 & com_default_proxy(
      const context::Model & model, context::Data & data, bool computeSubtreeComs = true)
    {
      return centerOfMass(model, data, computeSubtreeComs);
    }

    static context::Data::Matrix3x jacobian_subtree_com_kinematics_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      context::Model::JointIndex jointId)
    {
      context::Data::Matrix3x J(3, model.nv);
      J.setZero();
      jacobianSubtreeCenterOfMass(model, data, q, jointId, J);

      return J;
    }

    static context::Data::Matrix3x jacobian_subtree_com_proxy(
      const context::Model & model, context::Data & data, context::Model::JointIndex jointId)
    {
      context::Data::Matrix3x J(3, model.nv);
      J.setZero();
      jacobianSubtreeCenterOfMass(model, data, jointId, J);

      return J;
    }

    static context::Data::Matrix3x get_jacobian_subtree_com_proxy(
      const context::Model & model, context::Data & data, context::Model::JointIndex jointId)
    {
      context::Data::Matrix3x J(3, model.nv);
      J.setZero();
      getJacobianSubtreeCenterOfMass(model, data, jointId, J);

      return J;
    }

    void exposeCOM()
    {
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "computeTotalMass",
        (Scalar(*)(
          const context::Model &))&computeTotalMass<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model"), "Compute the total mass of the model and return it.");

      bp::def(
        "computeTotalMass",
        (Scalar(*)(
          const context::Model &,
          context::Data &))&computeTotalMass<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Compute the total mass of the model, put it in data.mass[0] and return it.");

      bp::def(
        "computeSubtreeMasses",
        (void (*)(
          const context::Model &,
          context::Data &))&computeSubtreeMasses<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Compute the mass of each kinematic subtree and store it in the vector data.mass.");

      bp::def(
        "centerOfMass", com_0_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("compute_subtree_coms") = true),
        "Compute the center of mass, putting the result in context::Data and return it."
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
        "subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "centerOfMass", com_1_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"),
         bp::arg("compute_subtree_coms") = true),
        "Computes the center of mass position and velocity by storing the result in context::Data. "
        "It returns the center of mass position expressed in the WORLD frame.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
        "subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "centerOfMass", com_2_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"), bp::arg("a"),
         bp::arg("compute_subtree_coms") = true),
        "Computes the center of mass position, velocity and acceleration by storing the result in "
        "context::Data. It returns the center of mass position expressed in the WORLD frame.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
        "subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "centerOfMass", com_level_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("kinematic_level"),
         bp::arg("compute_subtree_coms") = true),
        "Computes the center of mass position, velocity or acceleration of a given model according "
        "to the current kinematic values contained in data and the requested kinematic_level.\n"
        "If kinematic_level = POSITION, computes the CoM position, if kinematic_level = VELOCITY, "
        "also computes the CoM velocity and if kinematic_level = ACCELERATION, it also computes "
        "the CoM acceleration.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
        "subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "centerOfMass", com_default_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("compute_subtree_coms") = true),
        "Computes the center of mass position, velocity and acceleration of a given model "
        "according to the current kinematic values contained in data.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of "
        "the subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "jacobianCenterOfMass",
        (const context::Data::Matrix3x & (*)(const context::Model &, context::Data &,
                                             const Eigen::MatrixBase<VectorXs> &, bool))
          & jacobianCenterOfMass<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("compute_subtree_coms") = true),
        "Computes the Jacobian of the center of mass, puts the result in context::Data and return "
        "it.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of the "
        "subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "jacobianCenterOfMass",
        (const context::Data::Matrix3x & (*)(const context::Model &, context::Data &, bool))
          & jacobianCenterOfMass<Scalar, Options, JointCollectionDefaultTpl>,
        (bp::arg("model"), bp::arg("data"), bp::arg("compute_subtree_coms") = true),
        "Computes the Jacobian of the center of mass, puts the result in context::Data and "
        "return it.\n"
        "If compute_subtree_coms is True, the algorithm also computes the center of mass of "
        "the subtrees.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "jacobianSubtreeCenterOfMass", jacobian_subtree_com_kinematics_proxy,
        bp::args("model", "data", "q", "subtree_root_joint_id"),
        "Computes the Jacobian of the CoM of the given subtree (subtree_root_joint_id) "
        "expressed in the WORLD frame, according to the given joint configuration.");

      bp::def(
        "jacobianSubtreeCenterOfMass", jacobian_subtree_com_proxy,
        bp::args("model", "data", "subtree_root_joint_id"),
        "Computes the Jacobian of the CoM of the given subtree (subtree_root_joint_id) "
        "expressed in the WORLD frame, according to the given entries in data.");

      bp::def(
        "getJacobianSubtreeCenterOfMass", get_jacobian_subtree_com_proxy,
        bp::args("model", "data", "subtree_root_joint_id"),
        "Get the Jacobian of the CoM of the given subtree expressed in the world frame, "
        "according to the given entries in data. It assumes that jacobianCenterOfMass has "
        "been called first.");
    }

  } // namespace python
} // namespace pinocchio
