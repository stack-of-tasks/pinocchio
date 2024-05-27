//
// Copyright (c) 2016-2023 CNRS INRIA
//

#ifndef __pinocchio_python_algorithms_hpp__
#define __pinocchio_python_algorithms_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

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
    void exposeContactJacobian();
    void exposeContactDynamics();
    void exposeConstraintDynamics();
    void exposeConstraintDynamicsDerivatives();
    void exposeContactInverseDynamics();
    void exposeDelassus();
    void exposeCAT();
    void exposeJacobian();
    void exposeGeometryAlgo();
    void exposeKinematicRegressor();
    void exposeRegressor();
    void exposeCholesky();
    void exposeModelAlgo();
    void exposeImpulseDynamics();

    void exposeRNEADerivatives();
    void exposeABADerivatives();
    void exposeKinematicsDerivatives();
    void exposeFramesDerivatives();
    void exposeCentroidalDerivatives();
    void exposeImpulseDynamicsDerivatives();

    void exposeCones();

    void exposeContactSolvers();

    void exposeAlgorithms();

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithms_hpp__
