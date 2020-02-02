//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_algorithm_hpp__
#define __pinocchio_python_algorithm_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    void exposeJointsAlgo();
    void exposeABA();
    void exposeCRBA();
    void exposeCentroidal();
    void exposeRNEA();
    void exposeCOM();
    void exposeFramesAlgo();
    void exposeEnergy();
    void exposeKinematics();
    void exposeConstrainedDynamics();
    void exposeCAT();
    void exposeJacobian();
    void exposeGeometryAlgo();
    void exposeRegressor();
    void exposeCholesky();
    void exposeModelAlgo();
    
    void exposeRNEADerivatives();
    void exposeABADerivatives();
    void exposeKinematicsDerivatives();
    void exposeCentroidalDerivatives();

    void exposeAlgorithms();
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_hpp__

