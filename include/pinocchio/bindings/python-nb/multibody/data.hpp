// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/multibody.hpp"

#include "../fwd.hpp"
#include "../utils/comparable.hpp"
#include "../utils/deprecation.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/bind_vector.h>

#define NB_DATA_RW(NAME, DOC) def_rw(#NAME, &Data::NAME, DOC)
#define NB_DATA_RO(NAME, DOC) def_ro(#NAME, &Data::NAME, DOC)

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
constexpr char kContactCholDeprecateMsg[] = "Deprecated member. Use constraint_chol instead.";
template<class Data>
void exposeData(nb::module_ m)
{
  using namespace nb::literals;

  nb::class_<Data>(
    m, "Data",
    "Articulated rigid body data related to a Model.\n"
    "It contains all the data that can be modified by the Pinocchio algorithms.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<const Model &>(), "model"_a, "Constructs a data structure from a given model.")
    // --- joint data
    .NB_DATA_RW(
      joints, "Vector of JointData associated to each JointModel stored in the related model.")
    .NB_DATA_RW(q_in, "Input joint configuration vector.")
    .NB_DATA_RW(v_in, "Input joint velocity vector.")
    .NB_DATA_RW(a_in, "Input joint acceleration vector.")
    .NB_DATA_RW(tau_in, "Input joint torque vector.")
    // --- motion quantities
    .NB_DATA_RW(a, "Vector of joint accelerations expressed in the local frame of the joint.")
    .NB_DATA_RW(oa, "Joint spatial acceleration expressed at the origin of the world frame.")
    .NB_DATA_RW(
      a_gf,
      "Joint spatial acceleration containing also the contribution of the gravity acceleration.")
    .NB_DATA_RW(
      oa_gf,
      "Joint spatial acceleration containing also the contribution of the gravity acceleration, "
      "but expressed at the origin of the world frame.")
    .NB_DATA_RW(v, "Vector of joint velocities expressed in the local frame of the joint.")
    .NB_DATA_RW(ov, "Vector of joint velocities expressed at the origin of the world.")
    // --- force quantities
    .NB_DATA_RW(f, "Vector of body forces expressed in the local frame of the joint.")
    .NB_DATA_RW(of, "Vector of body forces expressed at the origin of the world.")
    .NB_DATA_RW(
      of_augmented,
      "Vector of body forces expressed at the origin of the world, in the context of lagrangian "
      "formulation.")
    .NB_DATA_RW(h, "Vector of spatial momenta expressed in the local frame of the joint.")
    .NB_DATA_RW(oh, "Vector of spatial momenta expressed at the origin of the world.")
    // --- placements
    .NB_DATA_RW(oMi, "Body absolute placement (wrt world).")
    .NB_DATA_RW(oMf, "Frames absolute placement (wrt world).")
    .NB_DATA_RW(liMi, "Body relative placement (wrt parent).")
    // --- RNEA / ABA outputs
    .NB_DATA_RW(tau, "Joint torques (output of RNEA).")
    .NB_DATA_RW(nle, "Non Linear Effects (output of nle algorithm).")
    .NB_DATA_RW(ddq, "Joint accelerations (output of ABA).")
    // --- inertia quantities
    .NB_DATA_RW(Ycrb, "Inertia of the sub-tree composite rigid body.")
    .NB_DATA_RW(
      oYcrb,
      "Composite Rigid Body Inertia of the sub-tree expressed in the WORLD coordinate system.")
    .NB_DATA_RW(Yaba, "Articulated Body Inertia of the sub-tree.")
    .NB_DATA_RW(
      oYaba, "Articulated Body Inertia of the sub-tree expressed in the WORLD coordinate system.")
    .NB_DATA_RW(
      oYaba_augmented,
      "Articulated Body Inertia matrix with constraint augmented inertia, expressed in the WORLD "
      "coordinate system.")
    .NB_DATA_RW(oL, "Acceleration propagator.")
    .NB_DATA_RW(oK, "Inverse articulated inertia.")
    // --- joint space matrices
    .NB_DATA_RW(M, "The joint space inertia matrix.")
    .NB_DATA_RW(Minv, "The inverse of the joint space inertia matrix.")
    .NB_DATA_RW(
      C, "The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v.")
    .NB_DATA_RW(g, "Vector of generalized gravity (dim model.nv).")
    .NB_DATA_RW(Fcrb, "Spatial forces set, used in CRBA.")
    .NB_DATA_RW(nvSubtree, "Dimension of the subtree motion space (for CRBA).")
    .NB_DATA_RW(U, "Joint inertia square root (upper triangle).")
    .NB_DATA_RW(D, "Diagonal of UDUT inertia decomposition.")
    // --- sparsity / row indexing
    .NB_DATA_RW(
      supports_fromRow,
      "Each element corresponds to the ordered list of indexes belonging to the supporting tree "
      "of the given index at the row level.")
    .NB_DATA_RW(start_idx_v_fromRow, "Starting index of the joint motion subspace.")
    .NB_DATA_RW(end_idx_v_fromRow, "End index of the joint motion subspace.")
    .NB_DATA_RW(parents_fromRow, "First previous non-zero row in M (used in Cholesky).")
    .NB_DATA_RW(
      mimic_parents_fromRow,
      "First previous non-zero row belonging to a mimic joint in M (used in Jacobian).")
    .NB_DATA_RW(
      non_mimic_parents_fromRow,
      "First previous non-zero row belonging to a non mimic joint in M (used in Jacobian).")
    .NB_DATA_RW(
      idx_vExtended_to_idx_v_fromRow,
      "Extended model mapping of the joint rows (idx_vExtended_to_idx_v_fromRow[idx_vExtended] = "
      "idx_v).")
    .NB_DATA_RW(nvSubtree_fromRow, "Subtree of the current row index (used in Cholesky).")
    .NB_DATA_RW(
      mimic_subtree_joint,
      "When mimic joints are in the tree, stores the index of the first non mimic child of each "
      "mimic joint.")
    // --- Jacobians
    .NB_DATA_RW(J, "Jacobian of joint placement.")
    .NB_DATA_RW(dJ, "Time variation of the Jacobian of joint placement (data.J).")
    .NB_DATA_RW(iMf, "Body placement wrt to algorithm end effector.")
    // --- inertia variations
    .NB_DATA_RW(Ivx, "Right variation of the inertia matrix.")
    .NB_DATA_RW(vxI, "Left variation of the inertia matrix.")
    .NB_DATA_RW(B, "Combined variations of the inertia matrix consistent with Christoffel symbols.")
    // --- centroidal quantities
    .NB_DATA_RW(Ag, "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
    .NB_DATA_RW(dAg, "Time derivative of the centroidal momentum matrix Ag.")
    .NB_DATA_RW(
      hg, "Centroidal momentum (expressed in the frame centered at the CoM and aligned with the "
          "world frame).")
    .NB_DATA_RW(
      dhg, "Centroidal momentum time derivative (expressed in the frame centered at the CoM and "
           "aligned with the world frame).")
    .NB_DATA_RW(Ig, "Centroidal Composite Rigid Body Inertia.")
    // --- CoM quantities
    .NB_DATA_RW(com, "CoM position of the subtree starting at joint index i.")
    .NB_DATA_RW(vcom, "CoM velocity of the subtree starting at joint index i.")
    .NB_DATA_RW(acom, "CoM acceleration of the subtree starting at joint index i.")
    .NB_DATA_RW(mass, "Mass of the subtree starting at joint index i.")
    .NB_DATA_RW(Jcom, "Jacobian of center of mass.")
    // --- derivatives
    .NB_DATA_RW(
      dAdq, "Variation of the spatial acceleration set with respect to the joint configuration.")
    .NB_DATA_RW(
      dAdv, "Variation of the spatial acceleration set with respect to the joint velocity.")
    .NB_DATA_RW(
      dHdq, "Variation of the spatial momenta set with respect to the joint configuration.")
    .NB_DATA_RW(dFdq, "Variation of the force set with respect to the joint configuration.")
    .NB_DATA_RW(dFdv, "Variation of the force set with respect to the joint velocity.")
    .NB_DATA_RW(dFda, "Variation of the force set with respect to the joint acceleration.")
    .NB_DATA_RW(
      dtau_dq,
      "Partial derivative of the joint torque vector with respect to the joint configuration.")
    .NB_DATA_RW(
      dtau_dv, "Partial derivative of the joint torque vector with respect to the joint velocity.")
    .NB_DATA_RW(
      ddq_dq, "Partial derivative of the joint acceleration vector with respect to the joint "
              "configuration.")
    .NB_DATA_RW(
      ddq_dv,
      "Partial derivative of the joint acceleration vector with respect to the joint velocity.")
    .NB_DATA_RW(
      ddq_dtau,
      "Partial derivative of the joint acceleration vector with respect to the joint torque.")
    .NB_DATA_RW(
      dvc_dq, "Partial derivative of the constraint velocity vector with respect to the joint "
              "configuration.")
    .NB_DATA_RW(
      dac_dq, "Partial derivative of the contact acceleration vector with respect to the joint "
              "configuration.")
    .NB_DATA_RW(
      dac_dv,
      "Partial derivative of the contact acceleration vector with respect to the joint velocity.")
    .NB_DATA_RW(
      dac_da, "Partial derivative of the contact acceleration vector with respect to the joint "
              "acceleration.")
    .NB_DATA_RW(osim, "Operational space inertia matrix.")
    // --- contact force derivatives (read-only)
    .NB_DATA_RO(
      dlambda_dq,
      "Partial derivative of the contact force vector with respect to the joint configuration.")
    .NB_DATA_RO(
      dlambda_dv,
      "Partial derivative of the contact force vector with respect to the joint velocity.")
    .NB_DATA_RO(
      dlambda_dtau, "Partial derivative of the contact force vector with respect to the torque.")
    // --- energies
    .NB_DATA_RW(kinetic_energy, "Kinetic energy in [J] computed by computeKineticEnergy.")
    .NB_DATA_RW(potential_energy, "Potential energy in [J] computed by computePotentialEnergy.")
    .NB_DATA_RW(
      mechanical_energy,
      "Mechanical energy in [J] of the system computed by computeMechanicalEnergy.")
    // --- contact dynamics
    .NB_DATA_RW(lambda_c, "Lagrange Multipliers linked to contact forces.")
    .NB_DATA_RW(impulse_c, "Lagrange Multipliers linked to contact impulses.")
    .def_prop_rw(
      "contact_chol",
      [](const Data & self) {
        deprecated_guard<kContactCholDeprecateMsg> guard;
        return self.contact_chol;
      },
      [](Data & self, const typename Data::ConstraintCholeskyDecomposition & constraint_chol) {
        deprecated_guard<kContactCholDeprecateMsg> guard;
        self.contact_chol = constraint_chol;
      })
    .NB_DATA_RW(
      primal_dual_contact_solution,
      "Right hand side vector when solving the contact dynamics KKT problem.")
    .NB_DATA_RW(
      lambda_c_prox,
      "Proximal Lagrange Multipliers used in the computation of the Forward Dynamics computations.")
    .NB_DATA_RW(primal_rhs_contact, "Primal RHS in contact dynamic equations.")
    // --- impact / regressors
    .NB_DATA_RW(dq_after, "Generalized velocity after the impact.")
    .NB_DATA_RW(staticRegressor, "Static regressor.")
    .NB_DATA_RW(jointTorqueRegressor, "Joint torque regressor.")
    .NB_DATA_RW(kineticEnergyRegressor, "Kinetic energy regressor.")
    .NB_DATA_RW(potentialEnergyRegressor, "Potential energy regressor.")
    // --- operators
    .def(ComparableVisitor<Data>());

  nb::bind_vector<std::vector<int>>(m, "StdVec_Int");
}
PINOCCHIO_PYTHON_NAMESPACE_END

#undef NB_DATA_RW
#undef NB_DATA_RO
