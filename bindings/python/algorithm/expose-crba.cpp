//
// Copyright (c) 2015-2016,2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace pinocchio
{
  namespace python
  {
    static Eigen::MatrixXd crba_proxy(const Model & model,
                                      Data & data,
                                      const Eigen::VectorXd & q)
    {
      data.M.fill(0);
      crba(model,data,q);
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      return data.M;
    }
    
    void exposeCRBA()
    {
      bp::def("crba",crba_proxy,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)"),
              "Computes CRBA, put the result in Data and return it.");
    }
    
  } // namespace python
} // namespace pinocchio
