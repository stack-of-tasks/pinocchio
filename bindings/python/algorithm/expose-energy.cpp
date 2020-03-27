//
// Copyright (c) 2015-2020 CNRS INRIA
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
      
      bp::def("computeKineticEnergy",
              &computeKineticEnergy<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","data","q","v"),
              "Computes the forward kinematics and the kinematic energy of the model for the "
              "given joint configuration and velocity given as input. The result is accessible through data.kinetic_energy.");
      
      bp::def("computeKineticEnergy",
              &computeKineticEnergy<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data"),
              "Computes the kinematic energy of the model for the "
              "given joint placement and velocity stored in data. The result is accessible through data.kinetic_energy.");
      
      bp::def("computePotentialEnergy",
              &computePotentialEnergy<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","data","q"),
              "Computes the potential energy of the model for the "
              "given the joint configuration given as input. The result is accessible through data.potential_energy.");
      
      bp::def("computePotentialEnergy",
              &computePotentialEnergy<double,0,JointCollectionDefaultTpl>,
              bp::args("model","data"),
              "Computes the potential energy of the model for the "
              "given joint placement stored in data. The result is accessible through data.potential_energy.");
    }
    
  } // namespace python
} // namespace pinocchio
