//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace pinocchio
{
  namespace python
  {
  
    void exposeRNEA()
    {
      using namespace Eigen;
      
      bp::def("rnea",
              &rnea<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)"),
              "Compute the RNEA, store the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("rnea",
              &rnea<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd,Force>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)",
                       "Acceleration a (size Model::nv)",
                       "Vector of external forces expressed in the local frame of each joint (size Model::njoints)"),
              "Compute the RNEA with external forces, store the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("nonLinearEffects",
              &nonLinearEffects<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), store the result in Data and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeGeneralizedGravity",
              &computeGeneralizedGravity<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)"),
              "Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store the result in data.g and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeStaticTorque",
              &computeStaticTorque<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Vector of external forces expressed in the local frame of each joint (size Model::njoints)"),
              "Computes the generalized static torque contribution g(q) - J.T f_ext of the Lagrangian dynamics, store the result in data.tau and return it.",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeCoriolisMatrix",
              &computeCoriolisMatrix<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Configuration q (size Model::nq)",
                       "Velocity v (size Model::nv)"),
              "Compute the Coriolis Matrix C(q,v) of the Lagrangian dynamics, store the result in data.C and return it.",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace pinocchio
