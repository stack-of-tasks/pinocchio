//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

namespace pinocchio
{
  namespace python
  {
    static void computeAllTerms_proxy(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v)
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
