import numpy as np
import pinocchio as pin
import hppfcl
from pinocchio.visualize import GepettoVisualizer

pin.switchToNumpyArray()

model = pin.Model()

geom_model = pin.GeometryModel()
geometries = [
    hppfcl.Capsule(0.1, 0.8),
    hppfcl.Sphere(0.5),
    hppfcl.Box(1, 1, 1),
    hppfcl.Cylinder(0.1, 1.0),
    hppfcl.Cone(0.5, 1.0),
]
for i, geom in enumerate(geometries):
    placement = pin.SE3(np.eye(3), np.array([i, 0, 0]))
    geom_obj = pin.GeometryObject("obj{}".format(i), 0, 0, geom, placement)
    color = np.random.uniform(0, 1, 4)
    color[3] = 1
    geom_obj.meshColor = color
    geom_model.addGeometryObject(geom_obj)

viz = GepettoVisualizer(
    model=model, collision_model=geom_model, visual_model=geom_model,
)

viz.initViewer()
viz.loadViewerModel("shapes")
viz.display(np.zeros(0))
