//
// Copyright (c) 2015-2021 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace pinocchio
{
  namespace python
  {
    static context::MatrixXs crba_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const Convention convention)
    {
      data.M.fill(context::Scalar(0));
      crba(model, data, q, convention);
      make_symmetric(data.M);
      return data.M;
    }

    void exposeCRBA()
    {
      bp::def(
        "crba", crba_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"),
         bp::arg("convention") = pinocchio::Convention::LOCAL),
        "Computes CRBA, store the result in Data and return it.\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\t convention: Convention to use");
    }

  } // namespace python
} // namespace pinocchio
