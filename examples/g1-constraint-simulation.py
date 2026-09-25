"""
Simulation of a g1 robot whose left arm is attached by a point anchor.
We also simulate the joint limits and the joint friction of the entire robot.

This example demonstrates how to:
  - Load a URDF model of a g1 robot
  - Construct a point anchor constraint
  - Construct a joint limit constraint
  - Construct a joint friction constraint
  - Compute the drift term for all of these constraints
  - Add Baumgarte stabilisation to the drift term for the position-level constraints
  - Construct a simulation loop to solve the constraint dynamics of this system
  - Set up the ADMM solver for solving hard constraint problems
  - Visualize the system's trajectory in Meshcat (optional)
"""

import os
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

# ─── 1. Kinematic / dynamic model ────────────────────────────────────────────

model_path = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
mesh_dir = model_path.parent.parent
urdf_filename = "g1_29dof_rev_1_0.urdf"
urdf_model_path = model_path / "g1_description/urdf" / urdf_filename
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)

# ─── 1. (Optional) Visualize the robot in Meshcat ────────────────────────────

has_viz = False
if pin.WITH_COLLISION:
    try:
        from pinocchio.visualize import MeshcatVisualizer

        viz = MeshcatVisualizer(model, collision_model, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()
        has_viz = True
        print("Meshcat viewer ready.")
    except Exception:
        print("Meshcat not available - running without visualization.")

# ─── 2. Initial configuration ─────────────────────────────────────────────────

q0 = pin.neutral(model)
q0[2] += 1.0  # for visualization purposes only
v0 = np.zeros(model.nv)  # zero initial velocity
zero_torque = np.zeros(model.nv)  # no external torques
dt = 1e-3  # time-step [s]

if has_viz:
    viz.display(q0)

# ─── 3. Build the point anchor, joint limit and joint friction constraints ────

constraint_models = []

# ── 3a. Point anchor on the left arm ─────────────────────────────────────────
# Anchor the left hand (left_rubber_hand frame) to its current world position.
# The PointAnchorConstraintModel enforces that a point fixed in the body frame
# of joint2 coincides with a point fixed in the frame of joint1.
#
# joint1 = universe (0):  placement1 gives the anchor location in world frame.
# joint2 = left_wrist_yaw_joint (23): placement2 is the hand frame offset in
#   joint 23's local frame, taken directly from the URDF frame definition.

anchor_frame_name = "left_rubber_hand"
anchor_fid = model.getFrameId(anchor_frame_name)
anchor_frame = model.frames[anchor_fid]
anchor_joint_id = anchor_frame.parentJoint  # 23 = left_wrist_yaw_joint

# Placement of the anchor point in the universe (world) frame at q0.
# Note: run forward kinematics once at q0 to get the world pose of the left hand.
data_init = model.createData(pin.DataAllocationOption.NO_TENSORS)
pin.forwardKinematics(model, data_init, q0)
pin.updateFramePlacements(model, data_init)
anchor_world_pos = data_init.oMf[anchor_fid].translation.copy()
anchor_placement_world = pin.SE3(np.eye(3), anchor_world_pos)

# Placement of the same point in joint 23's local frame (from the URDF).
anchor_placement_local = anchor_frame.placement

pacm = pin.PointAnchorConstraintModel(
    model,
    0,  # universe joint
    anchor_placement_world,
    anchor_joint_id,
    anchor_placement_local,
)
constraint_models.append(pin.ConstraintModel(pacm))

# Display a red ball at the anchor position in Meshcat.
if has_viz:
    import meshcat.geometry as mg
    import meshcat.transformations as mt

    viz.viewer["point_anchor"].set_object(
        mg.Sphere(0.05),
        mg.MeshLambertMaterial(color=0xFF0000),
    )
    viz.viewer["point_anchor"].set_transform(mt.translation_matrix(anchor_world_pos))

# ── 3b. Joint limit constraints ───────────────────────────────────────────────
# Apply joint limits to all actuated joints (skip joint 0 = universe and
# joint 1 = root free-flyer which has no meaningful position limits here).
actuated_joint_ids = list(range(2, model.njoints))
# Note: lower/upper limits which are +/- inf will be ignored by the constraint model.
# Note: the constructor of joint limit constraint expects the limits
# to be of size (nq,).
# It will automatically select the relevant entries based on the provided
# actuated_joint_ids.
assert model.lowerPositionLimit.shape == (model.nq,)
assert model.upperPositionLimit.shape == (model.nq,)
jlcm = pin.JointLimitConstraintModel(
    model,  #
    actuated_joint_ids,  #
    model.lowerPositionLimit,
    model.upperPositionLimit,
)
constraint_models.append(pin.ConstraintModel(jlcm))

# ── 3c. Joint friction constraints ───────────────────────────────────────────
# The g1 URDF does not specify dry-friction limits, so we set them manually
# as a fraction of the actuator effort limits.
# If the URDF had specified effort limits,
# we could then use model.[lower/upper]DryFrictionLimit directly.
joint_friction_coeff = 1.0
lower_friction = np.zeros(model.nv)
upper_friction = np.zeros(model.nv)
# skip universe and free-flyer
for joint in model.joints[2:]:
    lower_friction[joint.idx_v : joint.idx_v + joint.nv] = -joint_friction_coeff
    upper_friction[joint.idx_v : joint.idx_v + joint.nv] = joint_friction_coeff
fjcm = pin.JointFrictionConstraintModel(
    model, actuated_joint_ids, lower_friction, upper_friction
)
# Joint friction is velocity-dependent, so we must inform the time-step for
# proper scaling of the upper/lower limits
fjcm.setTimeStep(dt)
constraint_models.append(pin.ConstraintModel(fjcm))

total_residual_size = sum(cm.residualSize() for cm in constraint_models)
print(f"Number of constraints:          {len(constraint_models)}")
print(f"Total constraint residual size: {total_residual_size}")

# ─── 4. Setup and run the simulation loop ────────────────────────────────────

data = model.createData()
fext = [pin.Force.Zero() for _ in range(model.njoints)]

# Initialise constraint data.
constraint_datas = []
for cmodel in constraint_models:
    constraint_datas.append(cmodel.createData())

# Initialise the Cholesky factorisation structure (allocates memory based on
# the sparsity pattern; actual values are filled by chol.compute in the loop).
pin.crba(model, data, q0, pin.Convention.WORLD)
chol = pin.ConstraintCholeskyDecomposition(
    model, data, constraint_models, constraint_datas
)

# Initialise the constraint solver, its settings and the result container.
# Note: this simulation is hard to solve: the point anchor and the joint limits
# are very stiff constraints.
# We add anderson acceleration and set a relatively high mu_prox to help convergence.
solver = pin.ADMMConstraintSolver()
settings = pin.ADMMSolverSettings()
settings.max_iterations = 10000
settings.absolute_feasibility_tol = 1e-6
settings.relative_feasibility_tol = 1e-6
settings.absolute_complementarity_tol = 1e-6
settings.relative_complementarity_tol = 1e-6
settings.admm_update_rule = pin.ADMMUpdateRule.OSQP
settings.anderson_capacity = 10
settings.mu_prox = 1e-4
settings.solve_ncp = True
result = pin.ADMMSolverResult()

# Simulate for a few time-steps, solving the constraint problem at each step.
horizon = 1500
qs = [q0]
q = q0.copy()
v = v0.copy()
for t in range(horizon):
    # Data needs to be informed of the current state of the system for
    # downstream computations.
    data.q_in = q
    data.v_in = v
    data.tau_in = zero_torque

    # CRBA is required before building the Cholesky decomposition of the
    # Delassus matrix  G = J M⁻¹ Jᵀ.
    pin.crba(model, data, q, pin.Convention.WORLD)

    # Free velocity: what velocity the system would reach in one time-step
    # without any constraint forces.
    v_free = v + dt * pin.aba(model, data, q, v, zero_torque, fext)

    # Evaluate constraint Jacobians and residuals at the current state.
    for cmodel, cdata in zip(constraint_models, constraint_datas):
        cmodel.calc(model, data, cdata)

    # Cholesky decomposition of the Delassus matrix.
    chol.compute(model, data, constraint_models, constraint_datas, 1e-10)

    # DelassusCholeskyExpression wraps the Cholesky factors for efficient solves.
    delassus_expr = chol.getDelassusOperatorCholeskyExpression()

    # Constraint Jacobian and drift  g = J v_free.
    Jc = pin.getConstraintsJacobian(model, data, constraint_models, constraint_datas)
    g = Jc @ v_free
    # For position-level constraints (point anchor, joint limits) the drift can
    # also include the current position error to enforce Baumgarte stabilisation:
    #   g_pos = J v_free + kp * position_error / dt
    idx_cm = 0
    for cm, cd in zip(constraint_models, constraint_datas):
        cm_size = cm.residualSize()
        name = cm.shortname()
        if name == "PointAnchorConstraintModel":
            kp = cm.baumgarte_corrector_parameters.Kp
            g_anchor = g[idx_cm : idx_cm + cm_size]
            g_anchor += kp * cd.extract().constraint_position_error / dt
        if name == "JointLimitConstraintModel":
            kp = cm.baumgarte_corrector_parameters.Kp
            g_limits = g[idx_cm : idx_cm + cm_size]
            for i in range(cm_size):
                # Note: the constraint residual is negative when the position is
                # above the upper limit, and positive when below the lower limit.
                if cd.extract().constraint_residual[i] < 0.0:
                    # constraint violated, apply Baumgarte correction to push it
                    # back within limits.
                    g_limits[i] += kp * cd.extract().constraint_residual[i] / dt
                else:
                    # constraint not violated, the drift should include the velocity
                    # error to make sure no constraint impulse is applied when
                    # the joint is moving away from the limit.
                    g_limits[i] += cd.extract().constraint_residual[i] / dt
        idx_cm += cm_size

    # Solve the constraint problem with the ADMM solver.
    has_converged = solver.solve(
        delassus_expr, g, constraint_models, constraint_datas, settings, result
    )

    # Update configuration and velocity for the next time step by applying
    # the constraint impulses.
    constraint_impulses = result.retrieveConstraintImpulses()
    # convert impulses to forces/torques
    constraint_forces = (1.0 / dt) * constraint_impulses
    # map to generalised torques
    tau_constraints = Jc.T @ constraint_forces
    v_new = v + dt * pin.aba(model, data, q, v, zero_torque + tau_constraints, fext)
    q_new = pin.integrate(model, q, v_new * dt)

    q = q_new.copy()
    v = v_new.copy()
    qs.append(q)

    print(
        f"  step {t:3d}  |  converged: {result.converged}"
        f"  it: {result.iterations:4d}"
        f"  primal: {result.primal_feasibility:.2e}"
        f"  dual: {result.dual_feasibility:.2e}"
        f"  compl: {result.complementarity:.2e}"
    )


# ─── 5. (Optional) Visualize the robot trajectory ────────────────────────────
if has_viz:
    print("\nReplaying trajectory in Meshcat …")

    def subSample(qs: np.ndarray, duration: float, fps: float):
        nb_frames = len(qs)
        nb_subframes = int(duration * fps)
        if nb_frames < nb_subframes:
            return qs
        else:
            step = nb_frames // nb_subframes
            qs_sub = [qs[i] for i in range(0, nb_frames, step)]
            return qs_sub

    fps = 60.0
    dt_vis = 1.0 / fps
    qs = subSample(qs, dt * horizon, fps)
    for _ in range(3):
        for q_vis in qs:
            step_start = time.time()
            viz.display(q_vis)
            time_until_next_step = dt_vis - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        time.sleep(1.0)
