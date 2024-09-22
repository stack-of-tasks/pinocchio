# This examples shows how to load and move a robot in meshcat.
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat

import sys

import hppfcl as fcl
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

if tuple(map(int, fcl.__version__.split("."))) >= (3, 0, 0):
    with_octomap = fcl.WITH_OCTOMAP
else:
    with_octomap = False
if not with_octomap:
    print(
        "This example is skiped as HPP-FCL has not been compiled with octomap support."
    )

model = pin.Model()
collision_model = pin.GeometryModel()

octree = fcl.makeOctree(np.random.rand(1000, 3), 0.01)
octree_object = pin.GeometryObject("octree", 0, pin.SE3.Identity(), octree)
octree_object.meshColor[0] = 1.0
collision_model.addGeometryObject(octree_object)

visual_model = collision_model
viz = MeshcatVisualizer(model, collision_model, visual_model)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in
# a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()
viz.clearDefaultLights()
