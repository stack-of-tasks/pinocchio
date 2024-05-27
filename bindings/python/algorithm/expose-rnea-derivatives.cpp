//
// Copyright (c) 2018-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(context::Force) ForceAlignedVector;

    context::Data::MatrixXs computeGeneralizedGravityDerivatives(
      const context::Model & model, context::Data & data, const context::VectorXs & q)
    {
      context::Data::MatrixXs res(model.nv, model.nv);
      res.setZero();
      pinocchio::computeGeneralizedGravityDerivatives(model, data, q, res);
      return res;
    }

    context::Data::MatrixXs computeStaticTorqueDerivatives(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const ForceAlignedVector & fext)
    {
      context::Data::MatrixXs res(model.nv, model.nv);
      res.setZero();
      pinocchio::computeStaticTorqueDerivatives(model, data, q, fext, res);
      return res;
    }

    bp::tuple computeRNEADerivatives(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & a)
    {
      pinocchio::computeRNEADerivatives(model, data, q, v, a);
      make_symmetric(data.M);
      return bp::make_tuple(make_ref(data.dtau_dq), make_ref(data.dtau_dv), make_ref(data.M));
    }

    bp::tuple computeRNEADerivatives_fext(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & a,
      const ForceAlignedVector & fext)
    {
      pinocchio::computeRNEADerivatives(model, data, q, v, a, fext);
      make_symmetric(data.M);
      return bp::make_tuple(make_ref(data.dtau_dq), make_ref(data.dtau_dv), make_ref(data.M));
    }

    void exposeRNEADerivatives()
    {
      bp::def(
        "computeGeneralizedGravityDerivatives", computeGeneralizedGravityDerivatives,
        bp::args("model", "data", "q"),
        "Computes the partial derivative of the generalized gravity contribution\n"
        "with respect to the joint configuration.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "Returns: dtau_statique_dq\n");

      bp::def(
        "computeStaticTorqueDerivatives", computeStaticTorqueDerivatives,
        bp::args("model", "data", "q", "fext"),
        "Computes the partial derivative of the generalized gravity and external forces "
        "contributions (a.k.a static torque vector)\n"
        "with respect to the joint configuration.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n"
        "Returns: dtau_statique_dq\n");

      bp::def(
        "computeRNEADerivatives", computeRNEADerivatives, bp::args("model", "data", "q", "v", "a"),
        "Computes the RNEA partial derivatives, store the result in data.dtau_dq, "
        "data.dtau_dv and data.M (aka dtau_da)\n"
        "which correspond to the partial derivatives of the torque output with respect to "
        "the joint configuration,\n"
        "velocity and acceleration vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n\n"
        "Returns: (dtau_dq, dtau_dv, dtau_da)\n");

      bp::def(
        "computeRNEADerivatives", computeRNEADerivatives_fext,
        bp::args("model", "data", "q", "v", "a", "fext"),
        "Computes the RNEA partial derivatives with external contact foces,\n"
        "store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)\n"
        "which correspond to the partial derivatives of the torque output with respect to "
        "the joint configuration,\n"
        "velocity and acceleration vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n\n"
        "Returns: (dtau_dq, dtau_dv, dtau_da)\n");
    }

  } // namespace python
} // namespace pinocchio
