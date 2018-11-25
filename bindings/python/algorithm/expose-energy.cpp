//
// Copyright (c) 2015-2016,2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/energy.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeEnergy()
    {
      using namespace Eigen;
      
      bp::def("kineticEnergy",
              &kineticEnergy<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Joint velocity v (size Model::nv)",
                       "Update kinematics (bool)"),
              "Computes the kinematic energy of the model for the "
              "given joint configuration and velocity and stores the result "
              "in data.kinetic_energy. By default, the first order kinematic quantities of the model are updated.");
      
      bp::def("potentialEnergy",
              &potentialEnergy<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("Model","Data",
                       "Joint configuration q (size Model::nq)",
                       "Update kinematics (bool)"),
              "Computes the potential energy of the model for the "
              "given the joint configuration and stores the result "
              "in data.potential_energy. By default, the zero order kinematic quantities of the model are updated.");
    }
    
  } // namespace python
} // namespace pinocchio
