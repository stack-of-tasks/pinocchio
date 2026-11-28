//
// Copyright (c) 2018-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea-second-order-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
    typedef std::vector<context::Force> ForceAlignedVector;

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

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::tuple computeRNEASecondOrderDerivatives(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & a)
    {
      pinocchio::ComputeRNEASecondOrderDerivatives(model, data, q, v, a);
      // Row-major copies, so that the numpy arrays follow the C++ indexing.
      return bp::make_tuple(
        toRowMajorTensor(data.d2tau_dqdq), toRowMajorTensor(data.d2tau_dvdv),
        toRowMajorTensor(data.d2tau_dqdv), toRowMajorTensor(data.d2tau_dadq));
    }
#endif // ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE

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
        "Returns: dtau_statique_dq\n",
        mimic_not_supported_function<>(0));

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
        "Returns: dtau_statique_dq\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeRNEADerivatives", computeRNEADerivatives, bp::args("model", "data", "q", "v", "a"),
        "Computes the RNEA partial derivatives, store the result in data.dtau_dq, data.dtau_dv and "
        "data.M (aka dtau_da)\n"
        "which correspond to the partial derivatives of the torque output with respect to the "
        "joint configuration,\n"
        "velocity and acceleration vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n\n"
        "Returns: (dtau_dq, dtau_dv, dtau_da)\n",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeRNEADerivatives", computeRNEADerivatives_fext,
        bp::args("model", "data", "q", "v", "a", "fext"),
        "Computes the RNEA partial derivatives with external contact foces,\n"
        "store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)\n"
        "which correspond to the partial derivatives of the torque output with respect to the "
        "joint configuration,\n"
        "velocity and acceleration vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n\n"
        "Returns: (dtau_dq, dtau_dv, dtau_da)\n",
        mimic_not_supported_function<>(0));

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
      bp::def(
        "computeRNEASecondOrderDerivatives", computeRNEASecondOrderDerivatives,
        bp::args("model", "data", "q", "v", "a"),
        "Computes the second-order partial derivatives of the RNEA with respect to the joint "
        "configuration, velocity and acceleration, and stores them in data.d2tau_dqdq, "
        "data.d2tau_dvdv, data.d2tau_dqdv and data.d2tau_dadq.\n"
        "Each derivative is a (nv, nv, nv) array: d2tau_dxdy[i,j,k] = d^2 tau_i / (dx_j dy_k), "
        "where x_j is the inner and y_k the outer derivative variable.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model (created with DataAllocationOption.ALL)\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ta: the joint acceleration vector (size model.nv)\n\n"
        "Returns: (d2tau_dqdq, d2tau_dvdv, d2tau_dqdv, d2tau_dadq)\n",
        mimic_not_supported_function<>(0));
#endif // ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    }

  } // namespace python
} // namespace pinocchio
