#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
// algorithm/joints.cpp
void exposeJointsAlgo(nb::module_ m);
// algorithm/aba.cpp
void exposeABA(nb::module_ m);
// algorithm/crba.cpp
void exposeCRBA(nb::module_ m);
// algorithm/centroidal.cpp
void exposeCentroidal(nb::module_ m);
// algorithm/rnea.cpp
void exposeRNEA(nb::module_ m);
// algorithm/com.cpp
void exposeCOM(nb::module_ m);
// algorithm/frames.cpp
void exposeFramesAlgo(nb::module_ m);
// algorithm/energy.cpp
void exposeEnergy(nb::module_ m);
// algorithm/kinematics.cpp
void exposeKinematics(nb::module_ m);
// algorithm/contact-jacobian.cpp
void exposeContactJacobian(nb::module_ m);
// algorithm/contact-dynamics.cpp
void exposeContactDynamics(nb::module_ m);
// algorithm/constrained-dynamics.cpp
void exposeConstraintDynamics(nb::module_ m);
// algorithm/constrained-dynamics-derivatives.cpp
void exposeConstraintDynamicsDerivatives(nb::module_ m);
// algorithm/contact-inverse-dynamics.cpp
void exposeContactInverseDynamics(nb::module_ m);
// algorithm/delassus.cpp
void exposeDelassus(nb::module_ m);
// algorithm/cat.cpp
void exposeCAT(nb::module_ m);
// algorithm/jacobian.cpp
void exposeJacobian(nb::module_ m);
// algorithm/geometry.cpp
void exposeGeometryAlgo(nb::module_ m);
// algorithm/kinematic-regressor.cpp
void exposeKinematicRegressor(nb::module_ m);
// algorithm/regressor.cpp
void exposeRegressor(nb::module_ m);
// algorithm/cholesky.cpp
void exposeCholesky(nb::module_ m);
// algorithm/model.cpp
void exposeModelAlgo(nb::module_ m);
// algorithm/impulse-dynamics.cpp
void exposeImpulseDynamics(nb::module_ m);

// algorithm/rnea-derivatives.cpp
void exposeRNEADerivatives(nb::module_ m);
// algorithm/aba-derivatives.cpp
void exposeABADerivatives(nb::module_ m);
// algorithm/kinematics-derivatives.cpp
void exposeKinematicsDerivatives(nb::module_ m);
// algorithm/frames-derivatives.cpp
void exposeFramesDerivatives(nb::module_ m);
// algorithm/centroidal-derivatives.cpp
void exposeCentroidalDerivatives(nb::module_ m);
// algorithm/impulse-dynamics-derivatives.cpp
void exposeImpulseDynamicsDerivatives(nb::module_ m);

// algorithm/constraint-solvers.cpp
void exposeConstraintSolvers(nb::module_ m);

void exposeAlgorithms(nb::module_ m)
{
  exposeJointsAlgo(m);
  exposeABA(m);
  exposeCRBA(m);
  exposeCentroidal(m);
  exposeRNEA(m);
  exposeCOM(m);
  exposeFramesAlgo(m);
  exposeEnergy(m);
  exposeKinematics(m);
  exposeContactJacobian(m);
  exposeContactDynamics(m);
  exposeConstraintDynamics(m);
  exposeContactInverseDynamics(m);
  exposeDelassus(m);
  exposeCAT(m);
  exposeJacobian(m);
  exposeGeometryAlgo(m);
  exposeKinematicRegressor(m);
  exposeRegressor(m);
  exposeCholesky(m);
  exposeModelAlgo(m);
  exposeImpulseDynamics(m);

  exposeRNEADerivatives(m);
  exposeABADerivatives(m);
  exposeConstraintDynamicsDerivatives(m);
  exposeKinematicsDerivatives(m);
  exposeFramesDerivatives(m);
  exposeCentroidalDerivatives(m);
  exposeImpulseDynamicsDerivatives(m);

  exposeConstraintSolvers(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
