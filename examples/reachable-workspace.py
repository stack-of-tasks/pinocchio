import pinocchio as pin
import sys
from os.path import dirname, join, abspath
import time

from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir

urdf_filename = "panda.urdf"
urdf_model_path = join(join(model_path, "panda_description/urdf"), urdf_filename)

robot, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)
data = robot.createData()

# Start a new MeshCat server and client.
viz = MeshcatVisualizer(robot, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# # Load the robot in the viewer.
viz.loadViewerModel()
# Display a robot configuration.
q0 = (robot.upperPositionLimit.T + robot.lowerPositionLimit.T) / 2
horizon = 0.2  # seconds
frame = robot.getFrameId(
    robot.frames[-1].name
)  # for example the last frame of the robot
n_samples = 5
facet_dims = 2

verts, faces = pin.reachableWorkspace(robot, q0, horizon, frame, n_samples, facet_dims)
print("------------------- Display Vertex")

import meshcat.geometry as g

# meshcat triangulated mesh
poly = g.TriangularMeshGeometry(vertices=verts.T, faces=faces)
viz.viewer["poly"].set_object(
    poly, g.MeshBasicMaterial(color=0x000000, wireframe=True, linewidth=12, opacity=0.2)
)

while True:
    viz.display(q0)
    viz.viewer["poly"].set_object(
        poly,
        g.MeshBasicMaterial(color=0x000000, wireframe=True, linewidth=2, opacity=0.2),
    )

    time.sleep(1e-2)
