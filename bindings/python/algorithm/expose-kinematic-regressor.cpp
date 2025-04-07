//
// Copyright (c) 2020 INRIA
//
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeKinematicRegressor()
    {
      typedef context::Scalar Scalar;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "computeJointKinematicRegressor",
        (context::Data::Matrix6x (*)(
          const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame,
          const context::
            SE3 &))&computeJointKinematicRegressor<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data", "joint_id", "reference_frame", "placement"),
        "Computes the kinematic regressor that links the joint placements variations of the whole "
        "kinematic tree to the placement variation of the frame rigidly attached to the joint and "
        "given by its placement w.r.t. to the joint frame.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tjoint_id: index of the joint\n"
        "\treference_frame: reference frame in which the result is expressed (LOCAL, "
        "LOCAL_WORLD_ALIGNED or WORLD)\n"
        "\tplacement: relative placement to the joint frame\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeJointKinematicRegressor",
        (context::Data::Matrix6x (*)(
          const context::Model &, const context::Data &, const JointIndex,
          const ReferenceFrame))&computeJointKinematicRegressor<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data", "joint_id", "reference_frame"),
        "Computes the kinematic regressor that links the joint placement variations of the "
        "whole kinematic tree to the placement variation of the joint given as input.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tjoint_id: index of the joint\n"
        "\treference_frame: reference frame in which the result is expressed (LOCAL, "
        "LOCAL_WORLD_ALIGNED or WORLD)\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeFrameKinematicRegressor",
        (context::Data::Matrix6x (*)(
          const context::Model &, context::Data &, const FrameIndex,
          const ReferenceFrame))&computeFrameKinematicRegressor<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data", "frame_id", "reference_frame"),
        "Computes the kinematic regressor that links the joint placement variations of the "
        "whole kinematic tree to the placement variation of the frame given as input.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tframe_id: index of the frame\n"
        "\treference_frame: reference frame in which the result is expressed (LOCAL, "
        "LOCAL_WORLD_ALIGNED or WORLD)\n",
        mimic_not_supported_function<>(0));
    }

  } // namespace python
} // namespace pinocchio
