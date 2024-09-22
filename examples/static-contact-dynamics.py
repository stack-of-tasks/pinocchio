from os.path import abspath, dirname, join

import numpy as np
import pinocchio as pin

np.set_printoptions(linewidth=np.inf)

# ----- PROBLEM STATEMENT ------
#
# We want to find the contact forces and torques required to stand still at a
# configuration 'q0'.
# We assume 3D contacts at each of the feet
#
# The dynamic equation would look like:
#
# M*q_ddot + g(q) + C(q, q_dot) = tau + J^T*lambda --> (for the static case) --> g(q) =
# tau + Jc^T*lambda (1).

# ----- SOLVING STRATEGY ------

# Split the equation between the base link (_bl) joint and the rest of the joints (_j).
# That is,
#
#  | g_bl |   |  0  |   | Jc__feet_bl.T |   | l1 |
#  | g_j  | = | tau | + | Jc__feet_j.T  | * | l2 |    (2)
#                                           | l3 |
#                                           | l4 |

# First, find the contact forces l1, l2, l3, l4 (these are 3 dimensional) by solving for
# the first 6 rows of (2).
# That is,
#
# g_bl   = Jc__feet_bl.T * | l1 |
#                          | l2 |    (3)
#                          | l3 |
#                          | l4 |
#
# Thus we find the contact froces by computing the jacobian pseudoinverse,
#
# | l1 | = pinv(Jc__feet_bl.T) * g_bl  (4)
# | l2 |
# | l3 |
# | l4 |
#
# Now, we can find the necessary torques using the bottom rows in (2). That is,
#
#                             | l1 |
#  tau = g_j - Jc__feet_j.T * | l2 |    (5)
#                             | l3 |
#                             | l4 |

# ----- SOLUTION ------

# 0. DATA
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir
urdf_filename = "solo12.urdf"
urdf_model_path = join(join(model_path, "solo_description/robots"), urdf_filename)

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
data = model.createData()

q0 = np.array(
    [
        0.0,
        0.0,
        0.235,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.8,
        -1.6,
        0.0,
        -0.8,
        1.6,
        0.0,
        0.8,
        -1.6,
        0.0,
        -0.8,
        1.6,
    ]
)
v0 = np.zeros(model.nv)
a0 = np.zeros(model.nv)

# 1. GRAVITY TERM

# We compute the gravity terms by using the ID at desired configuration q0, with
# velocity and acceleration being 0. I.e., ID with a = v = 0.
g_grav = pin.rnea(model, data, q0, v0, a0)

g_bl = g_grav[:6]
g_j = g_grav[6:]

# 2. FIND CONTACTS

# First, we set the frame for our contacts. We assume the contacts are placed at the
# following 4 frames and they are 3D.
feet_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
feet_ids = [model.getFrameId(n) for n in feet_names]
bl_id = model.getFrameId("base_link")
ncontact = len(feet_names)

# Now, we need to find the contact Jacobians appearing in (1).
# These are the Jacobians that relate the joint velocity  to the velocity of each feet
Js__feet_q = [
    np.copy(pin.computeFrameJacobian(model, data, q0, id, pin.LOCAL)) for id in feet_ids
]

Js__feet_bl = [np.copy(J[:3, :6]) for J in Js__feet_q]

# Notice that we can write the equation above as an horizontal stack of Jacobians
# transposed and vertical stack of contact forces.
Jc__feet_bl_T = np.zeros([6, 3 * ncontact])
Jc__feet_bl_T[:, :] = np.vstack(Js__feet_bl).T

# Now I only need to do the pinv to compute the contact forces

ls = np.linalg.pinv(Jc__feet_bl_T) @ g_bl  # This is (3)

# Contact forces at local coordinates (at each foot coordinate)
ls__f = np.split(ls, ncontact)

pin.framesForwardKinematics(model, data, q0)

# Contact forces at base link frame
ls__bl = []
for l__f, foot_id in zip(ls__f, feet_ids):
    l_sp__f = pin.Force(l__f, np.zeros(3))
    l_sp__bl = data.oMf[bl_id].actInv(data.oMf[foot_id].act(l_sp__f))
    ls__bl.append(np.copy(l_sp__bl.vector))

print("\n--- CONTACT FORCES ---")
for l__f, foot_id, name in zip(ls__bl, feet_ids, feet_names):
    print(f"Contact force at foot {name} expressed at the BL is: {l__f}")

# Notice that if we add all the contact forces are equal to the g_grav
print(
    "Error between contact forces and gravity at base link: "
    f"{np.linalg.norm(g_bl - sum(ls__bl))}"
)

# 3. FIND TAU
# Find Jc__feet_j
Js_feet_j = [np.copy(J[:3, 6:]) for J in Js__feet_q]

Jc__feet_j_T = np.zeros([12, 3 * ncontact])
Jc__feet_j_T[:, :] = np.vstack(Js_feet_j).T

# Apply (5)
tau = g_j - Jc__feet_j_T @ ls

# 4. CROSS CHECKS

# INVERSE DYNAMICS
# We can compare this torques with the ones one would obtain when computing the ID
# considering the external forces in ls.
pin.framesForwardKinematics(model, data, q0)

joint_names = ["FL_KFE", "FR_KFE", "HL_KFE", "HR_KFE"]
joint_ids = [model.getJointId(n) for n in joint_names]

fs_ext = [pin.Force(np.zeros(6)) for _ in range(len(model.joints))]
for idx, joint in enumerate(model.joints):
    if joint.id in joint_ids:
        fext__bl = pin.Force(ls__bl[joint_ids.index(joint.id)])
        fs_ext[idx] = data.oMi[joint.id].actInv(data.oMf[bl_id].act(fext__bl))

tau_rnea = pin.rnea(model, data, q0, v0, a0, fs_ext)

print("\n--- ID: JOINT TORQUES ---")
print(f"Tau from RNEA:         {tau_rnea}")
print(f"Tau computed manually: {np.append(np.zeros(6), tau)}")
print(f"Tau error: {np.linalg.norm(np.append(np.zeros(6), tau) - tau_rnea)}")

# FORWARD DYNAMICS
# We can also check the results using FD. FD with the tau we got, q0 and v0, should give
# 0 acceleration and the contact forces.
Js_feet3d_q = [np.copy(J[:3, :]) for J in Js__feet_q]
acc = pin.forwardDynamics(
    model,
    data,
    q0,
    v0,
    np.append(np.zeros(6), tau),
    np.vstack(Js_feet3d_q),
    np.zeros(12),
)

print("\n--- FD: ACC. & CONTACT FORCES ---")
print(f"Norm of the FD acceleration: {np.linalg.norm(acc)}")
print(f"Contact forces manually: {ls}")
print(f"Contact forces FD: {data.lambda_c}")
print(f"Contact forces error: {np.linalg.norm(data.lambda_c - ls)}")
