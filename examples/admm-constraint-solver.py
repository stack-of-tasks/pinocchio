"""
Stack of two cubes solved with the ADMM constraint solver.

This example demonstrates how to:
  - Build a kinematic model with two free-floating cubes
  - Manually define 8 PointContactConstraintModel constraints:
      4 for the floor-cube 1 interaction (bottom corners of cube 1)
      4 for the cube 1-cube 2 interaction (corners at the shared face)
  - Build the Delassus operator via the Cholesky decomposition
  - Compute the constraint drift vector  g = J * v_free
  - Solve the constrained problem with the ADMM solver
  - Print convergence statistics
"""

import sys

import numpy as np
import pinocchio as pin

# ─── 1. Kinematic / dynamic model ────────────────────────────────────────────

model = pin.Model()
# Default gravity is already (0, 0, -9.81, 0, 0, 0); shown here for clarity.
model.gravity = pin.Motion(np.array([0.0, 0.0, -9.81, 0.0, 0.0, 0.0]))

box_size = 1.0  # edge length of each cube [m]
box_half = box_size / 2.0
box_mass = 1.0  # mass of each cube [kg]

box_inertia = pin.Inertia.FromBox(box_mass, box_size, box_size, box_size)

# Cube 1 - free-flyer joint branching from the universe (joint 0)
joint1_id = model.addJoint(
    0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "box1_joint"
)
model.appendBodyToJoint(joint1_id, box_inertia, pin.SE3.Identity())

# Cube 2 - free-flyer joint, also branching from the universe
joint2_id = model.addJoint(
    0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "box2_joint"
)
model.appendBodyToJoint(joint2_id, box_inertia, pin.SE3.Identity())

# ─── 2. Initial configuration ─────────────────────────────────────────────────

# Free-flyer q = [tx, ty, tz,  qx, qy, qz, qw]
# Cube 1 centre at z = box_half  (bottom face touching z = 0)
# Cube 2 centre at z = 3*box_half (bottom face touching the top of cube 1)
q_box1 = np.array([0.0, 0.0, box_half, 0.0, 0.0, 0.0, 1.0])
q_box2 = np.array([0.0, 0.0, 3.0 * box_half, 0.0, 0.0, 0.0, 1.0])
q0 = np.concatenate([q_box1, q_box2])

v0 = np.zeros(model.nv)  # zero initial velocity
zero_torque = np.zeros(model.nv)  # no external torques
dt = 1e-3  # time-step [s]

# ─── 3. Build the 8 contact constraints ──────────────────────────────────────

friction_coeff = 0.8


def make_corner_constraints(model, joint1_id, joint2_id, z1, z2):
    """
    Return 4 PointContactConstraintModel objects for the corners of a planar
    contact interface between two bodies.

    Parameters
    ----------
    joint1_id, joint2_id : joint indices of the two contacting bodies
    z1         : z-coordinate of the contact plane in jid1's local frame
                 (e.g. +box_half for the top face, 0.0 for the world floor)
    z2         : z-coordinate of the contact plane in jid2's local frame
                 (e.g. -box_half for the bottom face)
    """
    corners_xy = [
        np.array([+box_half, +box_half]),
        np.array([-box_half, +box_half]),
        np.array([-box_half, -box_half]),
        np.array([+box_half, -box_half]),
    ]
    cms = []
    for xy in corners_xy:
        p1 = np.array([xy[0], xy[1], z1])
        p2 = np.array([xy[0], xy[1], z2])
        joint1_placement = pin.SE3(np.eye(3), p1)
        joint2_placement = pin.SE3(np.eye(3), p2)
        cm = pin.PointContactConstraintModel(
            model, joint1_id, joint1_placement, joint2_id, joint2_placement
        )
        cm.setFriction(friction_coeff)
        cms.append(cm)
    return cms


constraint_models = []

# 4 constraints: floor (universe, jid=0) ↔ cube 1 bottom face
# - in universe frame: contact points are on the floor plane  z = 0
# - in box1 local frame: contact points are at the bottom corners  z = -box_half
for cm in make_corner_constraints(model, 0, joint1_id, 0.0, -box_half):
    constraint_models.append(pin.ConstraintModel(cm))

# 4 constraints: cube 1 top face ↔ cube 2 bottom face
# - in box1 local frame: top corners at  z = +box_half
# - in box2 local frame: bottom corners at  z = -box_half
for cm in make_corner_constraints(model, joint1_id, joint2_id, +box_half, -box_half):
    constraint_models.append(pin.ConstraintModel(cm))

total_residual_size = sum(cm.residualSize() for cm in constraint_models)
print(f"Number of constraints:          {len(constraint_models)}")
print(f"Total constraint residual size: {total_residual_size}")

# ─── 4. Setup and run the simulation loop ────────────────────────────────────

data = model.createData(pin.DataAllocationOption.NO_TENSORS)
fext = [pin.Force.Zero() for _ in range(model.njoints)]

# Initialise constraint data
constraint_datas = []
for cmodel in constraint_models:
    cdata = cmodel.createData()
    constraint_datas.append(cdata)

# Initialize constraint cholesky
chol = pin.ConstraintCholeskyDecomposition(
    model, data, constraint_models, constraint_datas
)

# Initialize constraint solver, its settings and result
solver = pin.ADMMConstraintSolver()
settings = pin.ADMMSolverSettings()
settings.max_iterations = 1000
settings.absolute_feasibility_tol = 1e-10
settings.relative_feasibility_tol = 1e-12
settings.absolute_complementarity_tol = 1e-10
settings.relative_complementarity_tol = 1e-12
settings.admm_update_rule = pin.ADMMUpdateRule.SPECTRAL
settings.mu_prox = 1e-6
settings.stat_record = (
    True  # per-iteration statistics. Turn off for faster solves if not needed.
)
settings.solve_ncp = True
result = pin.ADMMSolverResult()

# Simulate for a few time-steps, solving the constraint problem at each step.
horizon = 10
q = q0.copy()
v = v0.copy()
for t in range(horizon):
    # Data needs to be informed of the current state of the system for
    # downstream computations.
    data.q_in = q
    data.v_in = v
    data.tau_in = zero_torque

    # CRBA is required before building the Cholesky decomposition of the Delassus
    # matrix G = J M⁻¹ Jᵀ.
    # Note that other delassus operators may not necessarily require CRBA.
    pin.crba(model, data, q, pin.Convention.WORLD)

    # Free acceleration: velocity the system would reach in one
    # time-step without contacts.
    v_free = v + dt * pin.aba(model, data, q, v, zero_torque, fext)

    # In theory, the constraint models' placements need to be updated at each time step
    # of the simulation to reflect the current configuration q.
    # We don't do it in this specific example since the cubes are stable and the contact
    # positions don't change.
    # In a more general case, you would need to do something like:
    # for cmodel in constraint_models:
    #     cmodel.joint1_placement = ... # relative placement of the
    #                                     contact point in joint1's local frame.
    #     cmodel.joint2_placement = ... # relative placement of the
    #                                     contact point in joint2's local frame.

    # Run calc on constraint models
    for cmodel, cdata in zip(constraint_models, constraint_datas):
        cmodel.calc(model, data, cdata)

    # Cholesky decomposition of the Delassus matrix.
    chol.compute(model, data, constraint_models, constraint_datas, 1e-10)

    # DelassusCholeskyExpression wraps the Cholesky factors for efficient solves.
    delassus_expr = chol.getDelassusOperatorCholeskyExpression()

    # Constraint Jacobian and drift  g = J v_free.
    Jc = pin.getConstraintsJacobian(model, data, constraint_models, constraint_datas)
    g = Jc @ v_free

    print(f"Delassus matrix size:           {delassus_expr.matrix().shape}")
    print(f"Drift vector ‖g‖:               {np.linalg.norm(g):.4e}")

    # Solve the constraint problem with the ADMM solver.
    has_converged = solver.solve(
        delassus_expr, g, constraint_models, constraint_datas, settings, result
    )

    print()
    print(f"time step: {t}")
    print("── ADMM solver results ──────────────────────────────────────────────")
    print(f"  Converged:            {result.converged}")
    print(f"  Iterations:           {result.iterations}")
    print(f"  Primal feasibility:   {result.primal_feasibility:.4e}")
    print(f"  Dual feasibility:     {result.dual_feasibility:.4e}")
    print(f"  Complementarity:      {result.complementarity:.4e}")
    print(f"  Final rho:            {result.rho:.4e}")
    print("─────────────────────────────────────────────────────────────────────")

    constraint_impulses = result.retrieveConstraintImpulses()
    constraint_velocities = result.retrieveConstraintVelocities()
    print(f"\nConstraint impulses   ‖λ‖: {np.linalg.norm(constraint_impulses):.4e}")
    print(f"Constraint velocities ‖v‖: {np.linalg.norm(constraint_velocities):.4e}")

    if not has_converged:
        print("\nWarning: solver did not converge within the iteration budget.")
        sys.exit(1)

    # Update configuration and velocity for the next time step by
    # applying the constraint impulses.
    constraint_forces = (
        1.0 / dt
    ) * constraint_impulses  # convert impulses to forces/torques
    tau_constraints = Jc.T @ constraint_forces  # map to generalized torques
    v_new = v + dt * pin.aba(model, data, q, v, zero_torque + tau_constraints, fext)
    q_new = pin.integrate(model, q, v_new * dt)

    # The cubes should be stable so the configuration should not change much,
    # and the velocity should be close to zero:
    assert np.linalg.norm(v_new) < 1e-8
    assert np.linalg.norm(q_new - q) < 1e-8

    # Update q and v for the next iteration.
    q = q_new.copy()
    v = v_new.copy()
