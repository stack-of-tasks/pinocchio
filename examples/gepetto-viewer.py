# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test

import os
import sys
from pathlib import Path

import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer

# Load the URDF model.
model_path = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
mesh_dir = model_path.parent.parent
urdf_filename = "talos_reduced.urdf"
urdf_model_path = model_path / "talos_data/robots" / urdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
viz = GepettoVisualizer(model, collision_model, visual_model)

# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install gepetto-viewer"
    )
    print(err)
    sys.exit(0)

try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print(
        "Error while loading the viewer model. It seems you should start gepetto-viewer"
    )
    print(err)
    sys.exit(0)

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

# Display another robot.
viz2 = GepettoVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName="pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)
