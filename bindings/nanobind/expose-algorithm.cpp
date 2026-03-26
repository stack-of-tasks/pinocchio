#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
void exposeJointsAlgo(nb::module_ m);
void exposeABA(nb::module_ m);
void exposeCRBA(nb::module_ m);
void exposeCentroidal(nb::module_ m);
void exposeRNEA(nb::module_ m);
void exposeCOM(nb::module_ m);
void exposeFramesAlgo(nb::module_ m);
void exposeEnergy(nb::module_ m);
void exposeKinematics(nb::module_ m);
void exposeContactJacobian(nb::module_ m);
void exposeContactDynamics(nb::module_ m);
void exposeConstraintDynamics(nb::module_ m);
void exposeConstraintDynamicsDerivatives(nb::module_ m);
void exposeContactInverseDynamics(nb::module_ m);
void exposeDelassus(nb::module_ m);
void exposeCAT(nb::module_ m);
void exposeJacobian(nb::module_ m);
void exposeGeometryAlgo(nb::module_ m);
void exposeKinematicRegressor(nb::module_ m);
void exposeRegressor(nb::module_ m);
void exposeCholesky(nb::module_ m);
void exposeModelAlgo(nb::module_ m);
void exposeImpulseDynamics(nb::module_ m);

void exposeRNEADerivatives(nb::module_ m);
void exposeABADerivatives(nb::module_ m);
void exposeKinematicsDerivatives(nb::module_ m);
void exposeFramesDerivatives(nb::module_ m);
void exposeCentroidalDerivatives(nb::module_ m);
void exposeImpulseDynamicsDerivatives(nb::module_ m);

void exposeCones(nb::module_ m);

void exposeConstraintSolvers(nb::module_ m);

void exposeAlgorithms(nb::module_ m)
{
  exposeJointsAlgo(m);
  exposeABA(m);
  exposeCentroidal(m);
  exposeRNEA(m);
  exposeEnergy(m);
  exposeCAT(m);
  exposeGeometryAlgo(m);
  exposeCholesky(m);
  exposeABADerivatives(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
