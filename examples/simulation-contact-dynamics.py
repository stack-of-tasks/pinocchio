import math
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "talos_reduced.urdf"
urdf_model_path = model_path / "talos_data/robots" / urdf_filename
srdf_filename = "talos.srdf"
srdf_full_path = model_path / "talos_data/srdf" / srdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in
# a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=False)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
viz.loadViewerModel()

# Display a robot configuration.
pin.loadReferenceConfigurations(model, srdf_full_path)
q0 = model.referenceConfigurations["half_sitting"]
q_ref = pin.integrate(model, q0, 0.1 * np.random.rand(model.nv))
viz.display(q0)

feet_name = ["left_sole_link", "right_sole_link"]
frame_ids = [model.getFrameId(frame_name) for frame_name in feet_name]

v0 = np.zeros(model.nv)
v_ref = v0.copy()
data_sim = model.createData()
data_control = model.createData()

contact_models = []
contact_datas = []


for frame_id in frame_ids:
    frame = model.frames[frame_id]
    contact_model = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_6D, model, frame.parentJoint, frame.placement
    )

    contact_models.append(contact_model)
    contact_datas.append(contact_model.createData())

num_constraints = len(frame_ids)
contact_dim = 6 * num_constraints

pin.initConstraintDynamics(model, data_sim, contact_models)

t = 0
dt = 5e-3

S = np.zeros((model.nv - 6, model.nv))
S.T[6:, :] = np.eye(model.nv - 6)
Kp_posture = 30.0
Kv_posture = 0.05 * math.sqrt(Kp_posture)

q = q0.copy()
v = v0.copy()
tau = np.zeros(model.nv)

T = 5

while t <= T:
    print("t:", t)
    t += dt

    tic = time.time()
    J_constraint = np.zeros((contact_dim, model.nv))
    pin.computeJointJacobians(model, data_control, q)
    constraint_index = 0
    for k in range(num_constraints):
        contact_model = contact_models[k]
        J_constraint[constraint_index : constraint_index + 6, :] = pin.getFrameJacobian(
            model,
            data_control,
            contact_model.joint1_id,
            contact_model.joint1_placement,
            contact_model.reference_frame,
        )
        constraint_index += 6

    A = np.vstack((S, J_constraint))
    b = pin.rnea(model, data_control, q, v, np.zeros(model.nv))

    sol = np.linalg.lstsq(A.T, b, rcond=None)[0]
    tau = np.concatenate((np.zeros((6)), sol[: model.nv - 6]))

    tau[6:] += (
        -Kp_posture * (pin.difference(model, q_ref, q))[6:]
        - Kv_posture * (v - v_ref)[6:]
    )

    prox_settings = pin.ProximalSettings(1e-12, 1e-12, 10)
    a = pin.constraintDynamics(
        model, data_sim, q, v, tau, contact_models, contact_datas, prox_settings
    )
    print("a:", a.T)
    print("v:", v.T)
    print("constraint:", np.linalg.norm(J_constraint @ a))
    print("iter:", prox_settings.iter)

    v += a * dt
    q = pin.integrate(model, q, v * dt)

    viz.display(q)
    elapsed_time = time.time() - tic

    time.sleep(max(0, dt - elapsed_time))
    # input()
