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
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "computeKineticEnergy",
        &computeKineticEnergy<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v"),
        "Computes the forward kinematics and the kinematic energy of the system for the "
        "given joint configuration and velocity given as input. The result is accessible "
        "through data.kinetic_energy.");

      bp::def(
        "computeKineticEnergy", &computeKineticEnergy<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Computes the kinematic energy of the system for the "
        "given joint placement and velocity stored in data. The result is accessible through "
        "data.kinetic_energy.");

      bp::def(
        "computePotentialEnergy",
        &computePotentialEnergy<Scalar, Options, JointCollectionDefaultTpl, VectorXs>,
        bp::args("model", "data", "q"),
        "Computes the potential energy of the system for the "
        "given the joint configuration given as input. The result is accessible through "
        "data.potential_energy.");

      bp::def(
        "computePotentialEnergy",
        &computePotentialEnergy<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Computes the potential energy of the system for the "
        "given joint placement stored in data. The result is accessible through "
        "data.potential_energy.");

      bp::def(
        "computeMechanicalEnergy",
        &computeMechanicalEnergy<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs>,
        bp::args("model", "data", "q", "v"),
        "Computes the forward kinematics and the kinematic energy of the system for the "
        "given joint configuration and velocity given as input. The result is accessible through "
        "data.mechanical_energy.\n"
        "A byproduct of this function is the computation of both data.kinetic_energy and "
        "data.potential_energy too.");

      bp::def(
        "computeMechanicalEnergy",
        &computeMechanicalEnergy<Scalar, Options, JointCollectionDefaultTpl>,
        bp::args("model", "data"),
        "Computes the mechanical energy of the system for the "
        "given joint placement and velocity stored in data. The result is accessible through "
        "data.mechanical_energy.\n"
        "A byproduct of this function is the computation of both data.kinetic_energy and "
        "data.potential_energy too.");
    }

  } // namespace python
} // namespace pinocchio
