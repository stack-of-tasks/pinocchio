# This examples shows how to load several robots in panda3d_viewer.
# Note: this feature requires panda3d_viewer to be installed, this can be done using
# pip install --user panda3d_viewer

import pinocchio as pin
import sys

from os.path import dirname, join, abspath

# add path to the example-robot-data package
path = join(dirname(dirname(abspath(__file__))), 'models', 'others', 'python')
sys.path.append(path)
from example_robot_data import loadTalos, loadRomeo, loadICub, loadTiago
from example_robot_data import loadSolo, loadHyQ, loadHector

# import visualizer
from panda3d_viewer import Viewer
from pinocchio.visualize.panda3d_visualizer import Panda3dVisualizer

# open a GUI window
viewer = Viewer(window_title='python-pinocchio')

loaders = (loadTalos, loadRomeo, loadICub, loadTiago, loadSolo, loadHyQ,
           loadHector)

for i, load in enumerate(loaders):
    robot = load()
    robot.setVisualizer(Panda3dVisualizer())
    robot.initViewer(viewer=viewer)  # attach to a viewer's scene
    robot.loadViewerModel(group_name=robot.model.name)

    q = robot.q0[:]
    q[1] = 3 - i
    if load is loadRomeo:
        q[2] = 0.87

    robot.display(q)

viewer.join()
