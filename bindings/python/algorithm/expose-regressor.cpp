//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{
  namespace python
  {

    context::MatrixXs bodyRegressor_proxy(const context::Motion & v, const context::Motion & a)
    {
      return bodyRegressor(v, a);
    }

    context::MatrixXs jointBodyRegressor_proxy(
      const context::Model & model, context::Data & data, const JointIndex jointId)
    {
      return jointBodyRegressor(model, data, jointId);
    }

    context::MatrixXs frameBodyRegressor_proxy(
      const context::Model & model, context::Data & data, const FrameIndex frameId)
    {
      return frameBodyRegressor(model, data, frameId);
    }

    boost::python::tuple computeIndirectRegressors_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v)
    {
      auto result = computeIndirectRegressors(model, data, q, v);

      return boost::python::make_tuple(result.first, result.second);
    }

    void exposeRegressor()
    {
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "computeStaticRegressor",
        &computeStaticRegressor<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q"),
        "Compute the static regressor that links the inertia parameters of the system to its "
        "center of mass position,\n"
        "store the result in context::Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "bodyRegressor", &bodyRegressor_proxy, bp::args("velocity", "acceleration"),
        "Computes the regressor for the dynamic parameters of a single rigid body.\n"
        "The result is such that "
        "Ia + v x Iv = bodyRegressor(v,a) * I.toDynamicParameters()\n\n"
        "Parameters:\n"
        "\tvelocity: spatial velocity of the rigid body\n"
        "\tacceleration: spatial acceleration of the rigid body\n");

      bp::def(
        "jointBodyRegressor", &jointBodyRegressor_proxy, bp::args("model", "data", "joint_id"),
        "Compute the regressor for the dynamic parameters of a rigid body attached to a "
        "given joint.\n"
        "This algorithm assumes RNEA has been run to compute the acceleration and "
        "gravitational effects.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tjoint_id: index of the joint\n");

      bp::def(
        "frameBodyRegressor", &frameBodyRegressor_proxy, bp::args("model", "data", "frame_id"),
        "Computes the regressor for the dynamic parameters of a rigid body attached to a "
        "given frame.\n"
        "This algorithm assumes RNEA has been run to compute the acceleration and "
        "gravitational effects.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tframe_id: index of the frame\n");

      bp::def(
        "computeJointTorqueRegressor",
        &computeJointTorqueRegressor<
          Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v", "a"),
        "Compute the joint torque regressor that links the joint torque "
        "to the dynamic parameters of each link according to the current the robot motion,\n"
        "store the result in context::Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computeKineticEnergyRegressor",
        &computeKineticEnergyRegressor<
          Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v"),
        "Compute the kinetic energy regressor that links the kinetic energy"
        "to the dynamic parameters of each link according to the current the robot motion,\n"
        "store the result in context::Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computePotentialEnergyRegressor",
        &computePotentialEnergyRegressor<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q"),
        "Compute the potential energy regressor that links the potential energy"
        "to the dynamic parameters of each link according to the current the robot motion,\n"
        "store the result in context::Data and return it.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n",
        bp::return_value_policy<bp::return_by_value>());

      bp::def(
        "computeIndirectRegressors", &computeIndirectRegressors_proxy,
        bp::args("model", "data", "q", "v"),
        "Compute the indirect regressors of momentum and transposed coriolis matrix times velocity,\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n",
        bp::return_value_policy<bp::return_by_value>());

    }

  } // namespace python
} // namespace pinocchio
