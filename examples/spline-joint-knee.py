"""Knee spline joint driven by its physical flexion angle.

This example uses the SE(3) spline joint (``pin.JointModelSpline`` /
``pin.JointModelSplineBuilder``) to reproduce experimental knee kinematics: the
joint constrains its child (tibia) frame to a cumulative B-spline on SE(3)
defined by a list of control frames, a knot vector and a degree

source: Lee, S. H., & Terzopoulos, D. (2008).
Spline joints for multibody dynamics. In ACM SIGGRAPH 2008 papers (pp. 1-8).

The 8 control frames below were fit offline (Gauss-Newton, degree=3, n_ctrl=8)
to a planar OpenSim knee, where the tx/ty translations the rotation about Z (knee flexion)
are driven by knee flexion itself.
"""

import time
from pathlib import Path

import coal
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Bones: mesh folder, OpenSim (subject-specific) scale factors and display color.
MESH_DIR = Path(__file__).resolve().parent.parent / "models" / "biomechanics"
FEMUR_SCALE, TIBIA_SCALE = 1.17378, 1.12373
BONE_COLOR = np.array([0.96, 0.94, 0.86, 0.7])  # bone ivory, semi-transparent.

# Recenter so the femoral condyle (knee) sits at the world origin instead of the
# hip: the fixed femur and the spline joint are both lifted by the knee offset.
KNEE_FROM_HIP = np.array([-0.0046, -0.4644, 0.0])
RECENTER = pin.SE3(np.eye(3), -KNEE_FROM_HIP)

# Control frames from the offline 2D-knee regression, as (yaw [rad], tx, ty [m]).
CONTROL_FRAMES = [
    (-2.094400, -0.003842, -0.496205),
    (-1.815147, 0.001278, -0.489246),
    (-1.256640, 0.006243, -0.480186),
    (-0.418880, -0.002220, -0.466444),
    (0.418880, -0.008427, -0.463391),
    (1.256640, -0.005740, -0.461613),
    (1.815147, -0.006127, -0.464906),
    (2.094400, -0.006285, -0.464416),
]

def mesh_object(name, parent_joint, stl, scale, placement):
    """Build a mesh GeometryObject fixed to ``parent_joint`` at ``placement``."""
    mesh_scale = scale * np.ones(3)
    geom = coal.MeshLoader().load(str(stl), mesh_scale)
    obj = pin.GeometryObject(name, parent_joint, placement, geom, str(stl), mesh_scale)
    obj.meshColor = BONE_COLOR
    return obj

class PointTracer:
    """Trace a fixed point in Meshcat, keeping the dotted trail."""
    def __init__(self, viz, name, point=None, color=0xFF3030, size=0.004):
        self._world_to_femur = RECENTER.inverse()
        self._point = np.zeros(3) if point is None else np.asarray(point, float)
        self._material = mg.PointsMaterial(size=size, color=color)
        self._trail = []
        self._node = viz.viewer[f"tibia_traces/{name}"]
        self._node.set_transform(RECENTER.homogeneous)

    def add(self, tibia_pose):
        self._trail.append(self._world_to_femur.act(tibia_pose.act(self._point)))
        pts = np.asarray(self._trail, dtype=np.float32).T
        self._node.set_object(mg.Points(mg.PointsGeometry(pts), self._material))


def main():
    
    control_frames = [
    pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([tx, ty, 0.0]))
    for (yaw, tx, ty) in CONTROL_FRAMES
    ]

    # Knee-angle domain of the fitted spline (in degree).
    THETA_MIN_DEG, THETA_MAX_DEG = -120.0, 120.0

    knee = (
        pin.JointModelSplineBuilder()
        .withDegree(3) # degree of the splines, degree 3 means acceleration is continuous.
        .withControlFrameVector(control_frames) # list of SE3 transforms
        .withOpenUniformKnots(THETA_MIN_DEG, THETA_MAX_DEG) # Affine map from knee flexion angle (degrees) to spline parameter q in [0, 1].
        .build()
    )
    model = pin.Model()
    joint_id = model.addJoint(0, knee, RECENTER, "knee")

    # visual only code
    try:
        visual_model = pin.GeometryModel()
        visual_model.addGeometryObject(
            mesh_object("femur", 0, MESH_DIR / "femur_r.stl", FEMUR_SCALE, RECENTER)
        )
        visual_model.addGeometryObject(
            mesh_object(
                "tibia", joint_id, MESH_DIR / "tibia_r.stl", TIBIA_SCALE, pin.SE3.Identity()
            )
        )


        viz = MeshcatVisualizer(model, None, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()

    except ImportError as e:
        print("Error while initializing the viewer.")
        print(e)
        return

    # Trace tibia-frame points and show the tibia frame as a triad, updated every step.
    plateau =np.array([ 0.03100708, -0.03819558, -0.00347008])
    tracers = [
        PointTracer(viz, "origin", color=0xFF3030),
        PointTracer(viz, "plateau", plateau, color=0x3060FF),
    ]
    tibia_frame = viz.viewer["tibia_frame"]
    tibia_frame.set_object(mg.triad(0.08))

    # Sweep the physical knee angle (+10 deg hyperextension to -120 deg flexion),
    angles = np.linspace(10.0, -120.0, 120)
    angles = np.concatenate([angles, angles[::-1]])
    for _ in range(5):
        for theta_deg in angles:
            viz.display(np.array([theta_deg]))
            tibia_pose = viz.data.oMi[joint_id]
            for tracer in tracers:
                tracer.add(tibia_pose)
            tibia_frame.set_transform(tibia_pose.homogeneous)
            time.sleep(0.015)


if __name__ == "__main__":
    main()
