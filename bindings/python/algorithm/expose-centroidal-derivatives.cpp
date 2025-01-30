//
// Copyright (c) 2019-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {

    bp::tuple computeCentroidalDynamicsDerivatives_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v,
      const context::VectorXs & a)
    {
      typedef context::Data::Matrix6x Matrix6x;
      Matrix6x partialh_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_da(Matrix6x::Zero(6, model.nv));

      computeCentroidalDynamicsDerivatives(
        model, data, q, v, a, partialh_dq, partial_dq, partial_dv, partial_da);

      return bp::make_tuple(partialh_dq, partial_dq, partial_dv, partial_da);
    }

    bp::tuple
    getCentroidalDynamicsDerivatives_proxy(const context::Model & model, context::Data & data)
    {
      typedef context::Data::Matrix6x Matrix6x;

      Matrix6x partialh_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dq(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6, model.nv));
      Matrix6x partial_da(Matrix6x::Zero(6, model.nv));

      impl::getCentroidalDynamicsDerivatives(
        model, data, partialh_dq, partial_dq, partial_dv, partial_da);
      return bp::make_tuple(partialh_dq, partial_dq, partial_dv, partial_da);
    }

    void exposeCentroidalDerivatives()
    {
      using namespace Eigen;

      bp::def(
        "computeCentroidalDynamicsDerivatives", computeCentroidalDynamicsDerivatives_proxy,
        bp::args("model", "data", "q", "v", "a"),
        "Computes the analytical derivatives of the centroidal dynamics\n"
        "with respect to the joint configuration vector, velocity and acceleration.",
        mimic_not_supported_function<>(0));

      bp::def(
        "getCentroidalDynamicsDerivatives", getCentroidalDynamicsDerivatives_proxy,
        bp::args("model", "data"),
        "Retrive the analytical derivatives of the centroidal dynamics\n"
        "from the RNEA derivatives.\n"
        "pinocchio.computeRNEADerivatives should have been called first.",
        mimic_not_supported_function<>(0));
    }

  } // namespace python
} // namespace pinocchio
