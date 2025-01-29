import sys

import numpy as np
import pinocchio as pin

try:
    import coal
except ImportError:
    print("This example requires coal")
    sys.exit(0)
from pinocchio.visualize import GepettoVisualizer

model = pin.Model()

geom_model = pin.GeometryModel()
geometries = [
    coal.Capsule(0.1, 0.8),
    coal.Sphere(0.5),
    coal.Box(1, 1, 1),
    coal.Cylinder(0.1, 1.0),
    coal.Cone(0.5, 1.0),
]
for i, geom in enumerate(geometries):
    placement = pin.SE3(np.eye(3), np.array([i, 0, 0]))
    geom_obj = pin.GeometryObject(f"obj{i}", 0, 0, placement, geom)
    color = np.random.uniform(0, 1, 4)
    color[3] = 1
    geom_obj.meshColor = color
    geom_model.addGeometryObject(geom_obj)

viz = GepettoVisualizer(
    model=model,
    collision_model=geom_model,
    visual_model=geom_model,
)

# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as error:
    print(
        "Error while initializing the viewer. "
        "It seems you should install gepetto-viewer"
    )
    print(error)
    sys.exit(0)

try:
    viz.loadViewerModel("shapes")
except AttributeError as error:
    print(
        "Error while loading the viewer model. "
        "It seems you should start gepetto-viewer"
    )
    print(error)
    sys.exit(0)

viz.display(np.zeros(0))
