//
// Copyright (c) 2020 INRIA
//

#include <boost/python/tuple.hpp>

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    bp::tuple getFrameVelocityDerivatives_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Model::FrameIndex frame_id,
      ReferenceFrame rf)
    {
      typedef context::Data::Matrix6x Matrix6x;

      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));

      getFrameVelocityDerivatives(model, data, frame_id, rf, partial_dq, partial_dv);

      return bp::make_tuple(partial_dq, partial_dv);
    }

    bp::tuple getFrameVelocityDerivatives_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Model::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf)
    {
      typedef context::Data::Matrix6x Matrix6x;

      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));

      getFrameVelocityDerivatives(model, data, joint_id, placement, rf, partial_dq, partial_dv);

      return bp::make_tuple(partial_dq, partial_dv);
    }

    bp::tuple getFrameAccelerationDerivatives_proxy1(
      const context::Model & model,
      context::Data & data,
      const context::Model::FrameIndex frame_id,
      ReferenceFrame rf)
    {
      typedef context::Data::Matrix6x Matrix6x;

      Matrix6x v_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6, model.nv));

      getFrameAccelerationDerivatives(
        model, data, frame_id, rf, v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);

      return bp::make_tuple(v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);
    }

    bp::tuple getFrameAccelerationDerivatives_proxy2(
      const context::Model & model,
      context::Data & data,
      const context::Model::JointIndex joint_id,
      const context::SE3 & placement,
      ReferenceFrame rf)
    {
      typedef context::Data::Matrix6x Matrix6x;

      Matrix6x v_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x a_partial_da(Matrix6x::Zero(6, model.nv));

      getFrameAccelerationDerivatives(
        model, data, joint_id, placement, rf, v_partial_dq, a_partial_dq, a_partial_dv,
        a_partial_da);

      return bp::make_tuple(v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da);
    }

    void exposeFramesDerivatives()
    {
      using namespace Eigen;

      bp::def(
        "getFrameVelocityDerivatives", getFrameVelocityDerivatives_proxy1,
        bp::args("model", "data", "frame_id", "reference_frame"),
        "Computes the partial derivatives of the spatial velocity of a given frame with respect "
        "to\n"
        "the joint configuration and velocity and returns them as a tuple.\n"
        "The partial derivatives can be either expressed in the LOCAL frame of the joint, in the "
        "LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of "
        "reference_frame.\n"
        "You must first call computeForwardKinematicsDerivatives before calling this function.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tframe_id: index of the frame\n"
        "\treference_frame: reference frame in which the resulting derivatives are expressed\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "getFrameVelocityDerivatives", getFrameVelocityDerivatives_proxy2,
        bp::args("model", "data", "joint_id", "placement", "reference_frame"),
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
        "\treference_frame: reference frame in which the resulting derivatives are expressed\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "getFrameAccelerationDerivatives", getFrameAccelerationDerivatives_proxy1,
        bp::args("model", "data", "frame_id", "reference_frame"),
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
        "\treference_frame: reference frame in which the resulting derivatives are expressed\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "getFrameAccelerationDerivatives", getFrameAccelerationDerivatives_proxy2,
        bp::args("model", "data", "joint_id", "placement", "reference_frame"),
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
        "\treference_frame: reference frame in which the resulting derivatives are expressed\n",
        mimic_not_supported_function<>(0));
    }

  } // namespace python
} // namespace pinocchio
