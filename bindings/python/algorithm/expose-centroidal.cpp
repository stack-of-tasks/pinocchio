//
// Copyright (c) 2015-2016,2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/centroidal.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeCentroidal()
    {
      using namespace Eigen;
      
      bp::def("ccrba",
              &ccrba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("dccrba",
              dccrba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)"),
              "Computes the time derivative of the centroidal momentum matrix Ag in terms of q and v. It computes also the same information than ccrtba for the same price.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace pinocchio
