# This examples shows how to load several robots in panda3d_viewer.
# Note: this feature requires panda3d_viewer to be installed, this can be done using
# pip install panda3d_viewer
# ruff: noqa: E402

import sys
from os.path import abspath, dirname, join

# Add path to the example-robot-data package from git submodule.
# If you have it properly installed, there is no need for this sys.path thing.
path = join(
    dirname(dirname(abspath(__file__))), "models", "example-robot-data", "python"
)
sys.path.append(path)
from example_robot_data.robots_loader import (
    HectorLoader,
    HyQLoader,
    ICubLoader,
    RomeoLoader,
    Solo8Loader,
    TalosLoader,
    TiagoLoader,
)
from panda3d_viewer import Viewer
from pinocchio.visualize.panda3d_visualizer import Panda3dVisualizer

# Open a Panda3D GUI window
viewer = Viewer(window_title="python-pinocchio")

# These RobotLoader classes are defined in example_robot_data
loaders = (
    TalosLoader,
    RomeoLoader,
    ICubLoader,
    TiagoLoader,
    Solo8Loader,
    HyQLoader,
    HectorLoader,
)

for i, loader in enumerate(loaders):
    # The robot is loaded as a RobotWrapper object
    robot = loader().robot
    # Attach the robot to the viewer scene
    robot.setVisualizer(Panda3dVisualizer())
    robot.initViewer(viewer=viewer)
    robot.loadViewerModel(group_name=robot.model.name)

    q = robot.q0[:]
    q[1] = 3 - i
    if loader is RomeoLoader:
        q[2] = 0.87

    robot.display(q)

viewer.join()
