# This examples shows how to perform collision detection between the end-effector of a
# robot and a point cloud depicted as a Height Field
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat

import sys
import time
from pathlib import Path

import coal as fcl
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "panda.urdf"
urdf_model_path = model_path / "panda_description/urdf" / urdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)

# Add point clouds
num_points = 5000
points = np.random.rand(3, num_points)
point_cloud_placement = (
    pin.SE3.Identity()
)  # Placement of the point cloud wrt the WORLD frame
point_cloud_placement.translation = np.array([0.2, 0.2, -0.5])

X = points[0, :]
Y = points[1, :]
Z = points[2, :]

nx = 20
x_grid = np.linspace(0.0, 1.0, nx)
x_half_pad = 0.5 * (x_grid[1] - x_grid[0])
x_bins = np.digitize(X, x_grid + x_half_pad)
x_dim = x_grid[-1] - x_grid[0]

ny = 20
y_grid = np.linspace(0.0, 1.0, ny)
y_half_pad = 0.5 * (y_grid[1] - y_grid[0])
y_bins = np.digitize(Y, y_grid + y_half_pad)
y_dim = y_grid[-1] - y_grid[0]

point_bins = y_bins * nx + x_bins
heights = np.zeros((ny, nx))
np.maximum.at(heights.ravel(), point_bins, Z)

point_cloud = fcl.BVHModelOBBRSS()
point_cloud.beginModel(0, num_points)
point_cloud.addVertices(points.T)

height_field = fcl.HeightFieldOBBRSS(x_dim, y_dim, heights, min(Z))
height_field_placement = point_cloud_placement * pin.SE3(
    np.eye(3), 0.5 * np.array([x_grid[0] + x_grid[-1], y_grid[0] + y_grid[-1], 0.0])
)

go_point_cloud = pin.GeometryObject(
    "point_cloud", 0, point_cloud_placement, point_cloud
)
go_point_cloud.meshColor = np.ones(4)
collision_model.addGeometryObject(go_point_cloud)
visual_model.addGeometryObject(go_point_cloud)

go_height_field = pin.GeometryObject(
    "height_field", 0, height_field_placement, height_field
)
go_height_field.meshColor = np.ones(4)
height_field_collision_id = collision_model.addGeometryObject(go_height_field)
visual_model.addGeometryObject(go_height_field)

# Add colllision pair between the height field and the panda_hand geometry
panda_hand_collision_id = collision_model.getGeometryId("panda_hand_0")
go_panda_hand = collision_model.geometryObjects[panda_hand_collision_id]
go_panda_hand.geometry.buildConvexRepresentation(False)
go_panda_hand.geometry = (
    go_panda_hand.geometry.convex
)  # We need to work with the convex hull of the real mesh

collision_pair = pin.CollisionPair(height_field_collision_id, panda_hand_collision_id)
collision_model.addCollisionPair(collision_pair)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in
# a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
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
q0 = pin.neutral(model)
viz.display(q0)

is_collision = False
data = model.createData()
collision_data = collision_model.createData()
while not is_collision:
    q = pin.randomConfiguration(model)

    is_collision = pin.computeCollisions(
        model, data, collision_model, collision_data, q, True
    )

print("Found a configuration in collision:", q)
viz.display(q)
time.sleep(1.0)
