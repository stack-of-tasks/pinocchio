//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace pinocchio
{
  namespace python
  {

    static context::Data::Matrix6x get_frame_jacobian_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Data::FrameIndex frame_id,
      ReferenceFrame rf = LOCAL)
    {
      context::Data::Matrix6x J(6, model.nv);
      J.setZero();
      getFrameJacobian(model, data, frame_id, rf, J);

      return J;
    }

    static context::Data::Matrix6x get_frame_jacobian_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Data::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf = LOCAL)
    {
      context::Data::Matrix6x J(6, model.nv);
      J.setZero();
      getFrameJacobian(model, data, joint_id, placement, rf, J);
      return J;
    }

    static context::Data::Motion get_frame_velocity_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Data::FrameIndex frame_id,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameVelocity(model, data, frame_id, rf);
    }

    static context::Data::Motion get_frame_velocity_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Data::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameVelocity(model, data, joint_id, placement, rf);
    }

    static context::Data::Motion get_frame_acceleration_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Data::FrameIndex frame_id,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameAcceleration(model, data, frame_id, rf);
    }

    static context::Data::Motion get_frame_acceleration_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Data::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameAcceleration(model, data, joint_id, placement, rf);
    }

    static context::Data::Motion get_frame_classical_acceleration_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Data::FrameIndex frame_id,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameClassicalAcceleration(model, data, frame_id, rf);
    }

    static context::Data::Motion get_frame_classical_acceleration_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Data::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf = LOCAL)
    {
      return getFrameClassicalAcceleration(model, data, joint_id, placement, rf);
    }

    static context::Data::Matrix6x compute_frame_jacobian_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      context::Data::FrameIndex frame_id)
    {
      context::Data::Matrix6x J(6, model.nv);
      J.setZero();
      computeFrameJacobian(model, data, q, frame_id, J);

      return J;
    }

    static context::Data::Matrix6x compute_frame_jacobian_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      context::Data::FrameIndex frame_id,
      ReferenceFrame reference_frame)
    {
      context::Data::Matrix6x J(6, model.nv);
      J.setZero();
      computeFrameJacobian(model, data, q, frame_id, reference_frame, J);

      return J;
    }

    static context::Data::Matrix6x get_frame_jacobian_time_variation_proxy(
      const context::Model & model,
      context::Data & data,
      context::Data::FrameIndex jointId,
      ReferenceFrame rf)
    {
      context::Data::Matrix6x dJ(6, model.nv);
      dJ.setZero();
      getFrameJacobianTimeVariation(model, data, jointId, rf, dJ);

      return dJ;
    }

    static context::Data::Matrix6x frame_jacobian_time_variation_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::Data::FrameIndex frame_id,
      const ReferenceFrame rf)
    {
      computeJointJacobiansTimeVariation(model, data, q, v);
      updateFramePlacements(model, data);

      return get_frame_jacobian_time_variation_proxy(model, data, frame_id, rf);
    }

    void exposeFramesAlgo()
    {
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "updateFramePlacements", &updateFramePlacements<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Computes the placements of all the operational frames according to the current "
        "joint placement stored in data"
        "and puts the results in data.");

      bp::def(
        "updateFramePlacement", &updateFramePlacement<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data", "frame_id"),
        "Computes the placement of the given operational frame (frame_id) according to the "
        "current joint placement stored in data, stores the results in data and returns it.",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "getFrameVelocity", &get_frame_velocity_proxy1,
        (bp::arg("model"), bp::arg("data"), bp::arg("frame_id"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the spatial velocity of the frame expressed in the coordinate system given "
        "by reference_frame.\n"
        "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint "
        "spatial velocity stored in data.v");

      bp::def(
        "getFrameVelocity", &get_frame_velocity_proxy2,
        (bp::arg("model"), bp::arg("data"), bp::arg("joint_id"), bp::arg("placement"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the spatial velocity of the frame expressed in the coordinate system given "
        "by reference_frame.\n"
        "forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint "
        "spatial velocity stored in data.v");

      bp::def(
        "getFrameAcceleration", &get_frame_acceleration_proxy1,
        (bp::arg("model"), bp::arg("data"), bp::arg("frame_id"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the spatial acceleration of the frame expressed in the coordinate system "
        "given by reference_frame.\n"
        "forwardKinematics(model,data,q,v,a) should be called first to compute the joint "
        "spatial acceleration stored in data.a .");

      bp::def(
        "getFrameAcceleration", &get_frame_acceleration_proxy2,
        (bp::arg("model"), bp::arg("data"), bp::arg("joint_id"), bp::arg("placement"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the spatial acceleration of the frame expressed in the coordinate system "
        "given by reference_frame.\n"
        "forwardKinematics(model,data,q,v,a) should be called first to compute the joint "
        "spatial acceleration stored in data.a .");

      bp::def(
        "getFrameClassicalAcceleration", &get_frame_classical_acceleration_proxy1,
        (bp::arg("model"), bp::arg("data"), bp::arg("frame_id"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the \"classical\" acceleration of the frame expressed in the coordinate "
        "system given by reference_frame.\n"
        "forwardKinematics(model,data,q,v,a) should be called first to compute the joint "
        "spatial acceleration stored in data.a .");

      bp::def(
        "getFrameClassicalAcceleration", &get_frame_classical_acceleration_proxy2,
        (bp::arg("model"), bp::arg("data"), bp::arg("joint_id"), bp::arg("placement"),
         bp::arg("reference_frame") = LOCAL),
        "Returns the \"classical\" acceleration of the frame expressed in the coordinate "
        "system given by reference_frame.\n"
        "forwardKinematics(model,data,q,v,a) should be called first to compute the joint "
        "spatial acceleration stored in data.a .");

      bp::def(
        "framesForwardKinematics",
        &framesForwardKinematics<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q"),
        "Calls first the forwardKinematics(model,data,q) and then update the Frame placement "
        "quantities (data.oMf).");

      bp::def(
        "computeFrameJacobian",
        (context::Data::Matrix6x(*)(
          const context::Model &, context::Data &, const context::VectorXs &,
          context::Data::FrameIndex, ReferenceFrame))&compute_frame_jacobian_proxy,
        bp::args("model", "data", "q", "frame_id", "reference_frame"),
        "Computes the Jacobian of the frame given by its frame_id in the coordinate system "
        "given by reference_frame.\n");

      bp::def(
        "computeFrameJacobian",
        (context::Data::Matrix6x(*)(
          const context::Model &, context::Data &, const context::VectorXs &,
          context::Data::FrameIndex))&compute_frame_jacobian_proxy,
        bp::args("model", "data", "q", "frame_id"),
        "Computes the Jacobian of the frame given by its frame_id.\n"
        "The columns of the Jacobian are expressed in the coordinates system of the Frame itself.\n"
        "In other words, the velocity of the frame vF expressed in the local coordinate is given "
        "by J*v,"
        "where v is the joint velocity.");

      bp::def(
        "getFrameJacobian", &get_frame_jacobian_proxy1,
        bp::args("model", "data", "frame_id", "reference_frame"),
        "Computes the Jacobian of the frame given by its ID either in the LOCAL, "
        "LOCAL_WORLD_ALIGNED or the WORLD coordinates systems.\n"
        "In other words, the velocity of the frame vF expressed in the reference frame is "
        "given by J*v,"
        "where v is the joint velocity vector.\n"
        "remarks: computeJointJacobians(model,data,q) must have been called first.");

      bp::def(
        "getFrameJacobian", &get_frame_jacobian_proxy2,
        bp::args("model", "data", "joint_id", "placement", "reference_frame"),
        "Computes the Jacobian of the frame given by its placement with respect to the Joint "
        "frame and expressed the solution either in the LOCAL, LOCAL_WORLD_ALIGNED or the "
        "WORLD coordinates systems.\n"
        "In other words, the velocity of the frame vF expressed in the reference frame is "
        "given by J*v,"
        "where v is the joint velocity vector.\n\n"
        "remarks: computeJointJacobians(model,data,q) must have been called first.");

      bp::def(
        "frameJacobianTimeVariation", &frame_jacobian_time_variation_proxy,
        bp::args("model", "data", "q", "v", "frame_id", "reference_frame"),
        "Computes the Jacobian Time Variation of the frame given by its frame_id either in "
        "the reference frame provided by reference_frame.\n");

      bp::def(
        "getFrameJacobianTimeVariation", get_frame_jacobian_time_variation_proxy,
        bp::args("model", "data", "frame_id", "reference_frame"),
        "Returns the Jacobian time variation of the frame given by its frame_id either in "
        "the reference frame provided by reference_frame.\n"
        "You have to call computeJointJacobiansTimeVariation(model,data,q,v) and "
        "updateFramePlacements(model,data) first.");

      bp::def(
        "computeSupportedInertiaByFrame",
        &computeSupportedInertiaByFrame<double, 0, JointCollectionDefaultTpl>,
        bp::args("model", "data", "frame_id", "with_subtree"),
        "Computes the supported inertia by the frame (given by frame_id) and returns it.\n"
        "The supported inertia corresponds to the sum of the inertias of all the child frames "
        "(that belongs to the same joint body) and the child joints, if with_subtree=True.\n"
        "You must first call pinocchio::forwardKinematics to update placement values in data "
        "structure.");

      bp::def(
        "computeSupportedForceByFrame",
        &computeSupportedForceByFrame<double, 0, JointCollectionDefaultTpl>,
        bp::args("model", "data", "frame_id"),
        "Computes the supported force of the frame (given by frame_id) and returns it.\n"
        "The supported force corresponds to the sum of all the forces experienced after the "
        "given frame.\n"
        "You must first call pinocchio::rnea to update placement values in data structure.");
    }
  } // namespace python

} // namespace pinocchio
