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
    static context::MatrixXs
    crba_proxy(const context::Model & model, context::Data & data, const context::VectorXs & q)
    {
      data.M.fill(context::Scalar(0));
      crba(model, data, q);
      make_symmetric(data.M);
      return data.M;
    }

    namespace minimal
    {
      static context::MatrixXs
      crba_proxy(const context::Model & model, context::Data & data, const context::VectorXs & q)
      {
        data.M.fill(context::Scalar(0));
        ::pinocchio::minimal::crba(model, data, q);
        make_symmetric(data.M);
        return data.M;
      }
    } // namespace minimal

    void exposeCRBA()
    {
      {
        bp::scope current_scope = getOrCreatePythonNamespace("minimal");

        bp::def(
          "crba", minimal::crba_proxy, bp::args("model", "data", "q"),
          "Computes CRBA, store the result in Data and return it.\n"
          "Parameters:\n"
          "\tmodel: model of the kinematic tree\n"
          "\tdata: data related to the model\n"
          "\tq: the joint configuration vector (size model.nq)\n");
      }

      bp::def(
        "crba", crba_proxy, bp::args("model", "data", "q"),
        "Computes CRBA, store the result in Data and return it.\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n");
    }

  } // namespace python
} // namespace pinocchio
