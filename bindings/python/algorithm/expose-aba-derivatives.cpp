//
// Copyright (c) 2018-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba-second-order-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
    typedef std::vector<context::Force> ForceAlignedVector;

    bp::tuple computeABADerivatives(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & tau)
    {
      pinocchio::computeABADerivatives(model, data, q, v, tau);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    }

    bp::tuple computeABADerivatives_fext(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & tau,
      const ForceAlignedVector & fext)
    {
      pinocchio::computeABADerivatives(model, data, q, v, tau, fext);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    }

    bp::tuple computeABADerivatives_min(const context::Model & model, context::Data & data)
    {
      pinocchio::computeABADerivatives(model, data);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    }

    bp::tuple computeABADerivatives_min_fext(
      const context::Model & model, context::Data & data, const ForceAlignedVector & fext)
    {
      pinocchio::computeABADerivatives(model, data, fext);
      make_symmetric(data.Minv);
      return bp::make_tuple(make_ref(data.ddq_dq), make_ref(data.ddq_dv), make_ref(data.Minv));
    }

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::tuple computeABASecondOrderDerivatives(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & tau)
    {
      pinocchio::ComputeABASecondOrderDerivatives(model, data, q, v, tau);
      // Row-major copies, so that the numpy arrays follow the C++ indexing.
      return bp::make_tuple(
        toRowMajorTensor(data.d2ddq_dqdq), toRowMajorTensor(data.d2ddq_dvdv),
        toRowMajorTensor(data.d2ddq_dqdv), toRowMajorTensor(data.d2ddq_dtaudq));
    }
#endif // ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE

    void exposeABADerivatives()
    {
      using namespace Eigen;

      bp::def(
        "computeABADerivatives", computeABADerivatives, bp::args("model", "data", "q", "v", "tau"),
        "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv "
        "(aka ddq_dtau)\n"
        "which correspond to the partial derivatives of the joint acceleration vector output with "
        "respect to the joint configuration,\n"
        "velocity and torque vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ttau: the joint torque vector (size model.nv)\n\n"
        "Returns: (ddq_dq, ddq_dv, ddq_da)",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeABADerivatives", computeABADerivatives_fext,
        bp::args("model", "data", "q", "v", "tau", "fext"),
        "Computes the ABA derivatives with external contact foces,\n"
        "store the result in data.ddq_dq, data.ddq_dv and data.Minv (aka ddq_dtau)\n"
        "which correspond to the partial derivatives of the acceleration output with respect to "
        "the joint configuration,\n"
        "velocity and torque vectors.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ttau: the joint torque vector (size model.nv)\n"
        "\tfext: list of external forces expressed in the local frame of the joints (size "
        "model.njoints)\n\n"
        "Returns: (ddq_dq, ddq_dv, ddq_da)",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeABADerivatives", computeABADerivatives_min, bp::args("model", "data"),
        "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
        "which correspond to the partial derivatives of the joint acceleration vector output with "
        "respect to the joint configuration,\n"
        "velocity and torque vectors.\n"
        "By calling this function, the user assumes that pinocchio.optimized.aba has been called "
        "first, allowing to significantly reduce the computation timings by not recalculating "
        "intermediate results.",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeABADerivatives", computeABADerivatives_min_fext, bp::args("model", "data", "fext"),
        "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv\n"
        "which correspond to the partial derivatives of the joint acceleration vector output with "
        "respect to the joint configuration,\n"
        "velocity and torque vectors.\n"
        "By calling this function, the user assumes that pinocchio.optimized.aba has been called "
        "first, allowing to significantly reduce the computation timings by not recalculating "
        "intermediate results.",
        mimic_not_supported_function<>(0));

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
      bp::def(
        "computeABASecondOrderDerivatives", computeABASecondOrderDerivatives,
        bp::args("model", "data", "q", "v", "tau"),
        "Computes the second-order partial derivatives of the ABA (forward dynamics) with "
        "respect to the joint configuration, velocity and torque, and stores them in "
        "data.d2ddq_dqdq, data.d2ddq_dvdv, data.d2ddq_dqdv and data.d2ddq_dtaudq.\n"
        "Each derivative is a (nv, nv, nv) array: d2ddq_dxdy[i,j,k] = d^2 ddq_i / (dx_j dy_k), "
        "where x_j is the inner and y_k the outer derivative variable. In particular, "
        "d2ddq_dtaudq[i,j,k] = d Minv[i,j] / dq_k.\n\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model (created with DataAllocationOption.ALL)\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n"
        "\ttau: the joint torque vector (size model.nv)\n\n"
        "Returns: (d2ddq_dqdq, d2ddq_dvdv, d2ddq_dqdv, d2ddq_dtaudq)",
        mimic_not_supported_function<>(0));
#endif // ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    }
  } // namespace python
} // namespace pinocchio
