//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

namespace pinocchio
{
  namespace python
  {
    static void computeAllTerms_proxy(const context::Model & model,
                                      context::Data & data,
                                      const context::VectorXs & q,
                                      const context::VectorXs & v)
    {
      data.M.fill(0);
      computeAllTerms(model,data,q,v);
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeCAT()
    {
      bp::def("computeAllTerms",computeAllTerms_proxy,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute all the terms M, non linear effects and Jacobians in"
              "in the same loop and store the results in data.");
    }
  } // namespace python
} // namespace pinocchio
