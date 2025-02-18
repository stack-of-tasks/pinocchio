# This examples shows how to load and move a robot in meshcat.
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "romeo_small.urdf"
urdf_model_path = model_path / "romeo_description/urdf" / urdf_filename

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
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
# Color is needed here because the Romeo URDF doesn't contain any color, so the default
# color results in an invisible robot (alpha value set to 0).
viz.loadViewerModel(color=[0.0, 0.0, 0.0, 1.0])

# Display a robot configuration.
q0 = np.array(
    [
        0,
        0,
        0.840252,
        0,
        0,
        0,
        1,  # Free flyer
        0,
        0,
        -0.3490658,
        0.6981317,
        -0.3490658,
        0,  # left leg
        0,
        0,
        -0.3490658,
        0.6981317,
        -0.3490658,
        0,  # right leg
        0,  # chest
        1.5,
        0.6,
        -0.5,
        -1.05,
        -0.4,
        -0.3,
        -0.2,  # left arm
        0,
        0,
        0,
        0,  # head
        1.5,
        -0.6,
        0.5,
        1.05,
        -0.4,
        -0.3,
        -0.2,  # right arm
    ]
).T
viz.display(q0)

# Display another robot.
red_robot_viz = MeshcatVisualizer(model, collision_model, visual_model)
red_robot_viz.initViewer(viz.viewer)
red_robot_viz.loadViewerModel(rootNodeName="red_robot", color=[1.0, 0.0, 0.0, 0.5])
q = q0.copy()
q[1] = 1.0
red_robot_viz.display(q)
