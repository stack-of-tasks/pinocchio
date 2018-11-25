//
// Copyright (c) 2015-2018 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeAlgorithms()
    {
      exposeJointsAlgo();
      exposeABA();
      exposeCRBA();
      exposeCentroidal();
      exposeRNEA();
      exposeCOM();
      exposeFramesAlgo();
      exposeEnergy();
      exposeKinematics();
      exposeDynamics();
      exposeCAT();
      exposeJacobian();
      exposeGeometryAlgo();
      exposeRegressor();
      
      // expose derivative version of the algorithms
      exposeRNEADerivatives();
      exposeABADerivatives();
      exposeKinematicsDerivatives();
    }
    
  } // namespace python
} // namespace pinocchio
