//
// Copyright (c) 2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeRegressor()
    {
      using namespace Eigen;

      bp::def("computeStaticRegressor",
              &regressor::computeStaticRegressor<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the static regressor that links the inertia parameters of the system to its center of mass position\n"
              ",put the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace pinocchio
