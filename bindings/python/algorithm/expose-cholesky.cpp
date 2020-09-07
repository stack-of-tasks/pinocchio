//
// Copyright (c) 2019 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/algorithm/cholesky.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeCholesky()
    {
      using namespace Eigen;
      using namespace pinocchio::cholesky;

      {
        // using the cholesky scope
        bp::scope current_scope = getOrCreatePythonNamespace("cholesky");
        
        bp::def("decompose",
                &decompose<double,0,JointCollectionDefaultTpl>,
                bp::args("Model","Data"),
                "Computes the Cholesky decomposition of the joint space inertia matrix M contained in data.\n"
                "The upper triangular part of data.M should have been filled first by calling crba, or any related algorithms.",
                bp::return_value_policy<bp::return_by_value>());
        
        bp::def("solve",
                &solve<double,0,JointCollectionDefaultTpl,VectorXd>,
                bp::args("Model","Data","v"),
                "Returns the solution \f$x\f$ of \f$ M x = y \f$ using the Cholesky decomposition stored in data given the entry \f$ y \f$.",
                bp::return_value_policy<bp::return_by_value>());
        
        bp::def("computeMinv",
                &pinocchio::cholesky::computeMinv<double,0,JointCollectionDefaultTpl>,
                bp::args("Model","Data"),
                "Returns the inverse of the inverse of the joint space inertia matrix using the results of the Cholesky decomposition\n"
                "performed by cholesky.decompose. The result is stored in data.Minv.",
                bp::return_value_policy<bp::return_by_value>());
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
