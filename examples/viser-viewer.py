# This examples shows how to load and move a robot in Viser.
# Note: this feature requires Viser to be installed, this can be done using
# pip install --user viser

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
# urdf_filename = "talos_reduced.urdf"
# urdf_model_path = join(join(model_path,"talos_data/robots"),urdf_filename)
urdf_filename = "solo.urdf"
urdf_model_path = model_path / "solo_description/robots" / urdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)

# Start a new Viser server and client.
try:
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install viser")
    print(err)
    sys.exit(0)

# Create a convex shape from solo main body
mesh = visual_model.geometryObjects[0].geometry
mesh.buildConvexRepresentation(True)
convex = mesh.convex

# Place the convex object on the scene and display it
if convex is not None:
    placement = pin.SE3.Identity()
    placement.translation[0] = 2.0
    geometry = pin.GeometryObject("convex", 0, placement, convex)
    geometry.meshColor = np.ones(4)
    visual_model.addGeometryObject(geometry)
    # After modifying the visual_model we must rebuild
    # associated data inside the visualizer
    viz.rebuildData()

# Load the robot in the viewer.
viz.loadViewerModel()

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)
viz.displayVisuals(True)

# Display another robot.
viz2 = ViserVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName="pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)

# standing config
q1 = np.array(
    [0.0, 0.0, 0.235, 0.0, 0.0, 0.0, 1.0, 0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6]
)

v0 = np.random.randn(model.nv) * 2
data = viz.data
pin.forwardKinematics(model, data, q1, v0)
frame_id = model.getFrameId("HR_FOOT")
viz.display()

model.gravity.linear[:] = 0.0
dt = 0.01


def sim_loop():
    tau0 = np.zeros(model.nv)
    qs = [q1]
    vs = [v0]
    nsteps = 100
    for i in range(nsteps):
        q = qs[i]
        v = vs[i]
        a1 = pin.aba(model, data, q, v, tau0)
        vnext = v + dt * a1
        qnext = pin.integrate(model, q, dt * vnext)
        qs.append(qnext)
        vs.append(vnext)
        viz.display(qnext)
    return qs, vs


qs, vs = sim_loop()

with viz.create_video_ctx("../leap.mp4"):
    viz.play(qs, dt)
