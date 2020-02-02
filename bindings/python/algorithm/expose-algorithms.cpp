//
// Copyright (c) 2015-2020 CNRS INRIA
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
      exposeConstrainedDynamics();
      exposeCAT();
      exposeJacobian();
      exposeGeometryAlgo();
      exposeRegressor();
      exposeCholesky();
      exposeModelAlgo();
      exposeCentroidalDerivatives();
      
      // expose derivative version of the algorithms
      exposeRNEADerivatives();
      exposeABADerivatives();
      exposeKinematicsDerivatives();
    }
    
  } // namespace python
} // namespace pinocchio
