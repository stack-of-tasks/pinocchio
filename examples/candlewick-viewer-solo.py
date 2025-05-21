"""
See also: meshcat-viewer-solo.py
"""

import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from candlewick.multibody import Visualizer, VisualizerConfig

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "solo12.urdf"
urdf_model_path = model_path / "solo_description/robots" / urdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
visual_model: pin.GeometryModel

config = VisualizerConfig()
config.width = 1920
config.height = 1080


def ground(xy):
    return (
        np.sin(xy[0] * 3) / 5
        + np.cos(xy[1] ** 2 * 3) / 20
        + np.sin(xy[1] * xy[0] * 5) / 10
    )


def vizGround(elevation_fn, space, name="ground", color=np.array([1.0, 1.0, 0.6, 0.8])):
    xg = np.arange(-2, 2, space)
    nx = xg.shape[0]
    xy_g = np.meshgrid(xg, xg)
    xy_g = np.stack(xy_g)
    elev_g = np.zeros((nx, nx))
    elev_g[:, :] = elevation_fn(xy_g)

    sx = xg[-1] - xg[0]
    sy = xg[-1] - xg[0]
    elev_g[:, :] = elev_g[::-1, :]
    import coal

    heightField = coal.HeightFieldAABB(sx, sy, elev_g, np.min(elev_g))
    pl = pin.SE3.Identity()
    obj = pin.GeometryObject(name, 0, pl, heightField)
    obj.meshColor[:] = color
    obj.overrideMaterial = True
    visual_model.addGeometryObject(obj)


colorrgb = [128, 149, 255, 200]
colorrgb = np.array(colorrgb) / 255.0
vizGround(ground, 0.02, color=colorrgb)

viz = Visualizer(config, model, geomModel=visual_model)
print(
    "Candlewick visualizer: opened on device driver", viz.renderer.device.driverName()
)

q_ref = np.array(
    [
        [0.09906518],
        [0.20099078],
        [0.32502457],
        [0.19414175],
        [-0.00524735],
        [-0.97855773],
        [0.06860185],
        [0.00968163],
        [0.60963582],
        [-1.61206407],
        [-0.02543309],
        [0.66709088],
        [-1.50870083],
        [0.32405118],
        [-1.15305599],
        [1.56867351],
        [-0.39097222],
        [-1.29675892],
        [1.39741073],
    ]
)

while not viz.shouldExit:
    viz.display(q_ref)

time.sleep(1.0)
