"""
Pose a Solo-12 robot on a surface defined through a function and displayed through an hppfcl.HeightField.
"""

import numpy as np
import pinocchio as pin

from pinocchio.visualize import MeshcatVisualizer
from example_robot_data import load

robot = load("solo12")

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


model = robot.model
vizer = MeshcatVisualizer(model, robot.collision_model, robot.visual_model)
vizer.initViewer(loadModel=True)


def ground(xy):
    return (
        np.sin(xy[0] * 3) / 5
        + np.cos(xy[1] ** 2 * 3) / 20
        + np.sin(xy[1] * xy[0] * 5) / 10
    )


def vizGround(viz, elevation_fn, space, name="ground", color=[1.0, 1.0, 0.6, 0.8]):
    xg = np.arange(-2, 2, space)
    nx = xg.shape[0]
    xy_g = np.meshgrid(xg, xg)
    xy_g = np.stack(xy_g)
    elev_g = np.zeros((nx, nx))
    elev_g[:, :] = elevation_fn(xy_g)

    sx = xg[-1] - xg[0]
    sy = xg[-1] - xg[0]
    elev_g[:, :] = elev_g[::-1, :]
    import hppfcl

    heightField = hppfcl.HeightFieldAABB(sx, sy, elev_g, np.min(elev_g))
    pl = pin.SE3.Identity()
    obj = pin.GeometryObject("ground", 0, pl, heightField)
    obj.meshColor[:] = color
    viz.addGeometryObject(obj)
    viz.viewer.open()


colorrgb = [128, 149, 255, 200]
colorrgb = np.array(colorrgb) / 255.0
vizGround(vizer, ground, 0.02, color=colorrgb)

vizer.display(q_ref)
