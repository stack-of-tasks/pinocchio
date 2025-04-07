//
// Copyright (c) 2018-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(context::Force) ForceAlignedVector;

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

    void exposeABADerivatives()
    {
      using namespace Eigen;

      bp::def(
        "computeABADerivatives", computeABADerivatives, bp::args("model", "data", "q", "v", "tau"),
        "Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and "
        "data.Minv (aka ddq_dtau)\n"
        "which correspond to the partial derivatives of the joint acceleration vector output "
        "with respect to the joint configuration,\n"
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
        "which correspond to the partial derivatives of the acceleration output with respect "
        "to the joint configuration,\n"
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
    }
  } // namespace python
} // namespace pinocchio
