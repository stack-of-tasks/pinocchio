"""Knee and patella spline joints driven by the physical knee flexion angle.

This example uses the SE(3) spline joint (``pin.JointModelSpline`` /
``pin.JointModelSplineBuilder``) to reproduce experimental knee kinematics: the
joint constrains its child (tibia) frame to a cumulative B-spline on SE(3)
defined by a list of control frames, a knot vector and a degree

source: Lee, S. H., & Terzopoulos, D. (2008).
Spline joints for multibody dynamics. In ACM SIGGRAPH 2008 papers (pp. 1-8).

The patella is a second spline joint, child of the tibia, that mimics the knee:
it has no degree of freedom of its own, its spline parameter is the knee angle
Its motion subspace depends on the knee angle.

All the control frames were fit offline (least squares on SE(3), degree=3,
n_ctrl=8) to the OpenSim model leg6dof9musc, where the tibia (in the femur) and
the patella (in the tibia) translate along x/y and rotate about z as functions
of knee flexion.
"""

import time
from pathlib import Path

import coal

try:
    import meshcat.geometry as mg
except ImportError as err:
    import sys

    print("Error while importing meshcat. It seems you should install Python meshcat")
    print(err)
    sys.exit(0)
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Bones: mesh folder, OpenSim (subject-specific) scale factors and display color.
MESH_DIR = Path(__file__).resolve().parent.parent / "models" / "biomechanics"
FEMUR_SCALE, TIBIA_SCALE, PATELLA_SCALE = 1.17378, 1.12373, 1.09544
BONE_COLOR = np.array([0.96, 0.94, 0.86, 0.7])  # bone ivory, semi-transparent.

# Recenter so the femoral condyle (knee) sits at the world origin instead of the
# hip: the fixed femur and the spline joint are both lifted by the knee offset.
KNEE_FROM_HIP = np.array([-0.0046, -0.4644, 0.0])
RECENTER = pin.SE3(np.eye(3), -KNEE_FROM_HIP)

# Control frames from the offline 2D-knee regression, as (yaw [rad], tx, ty [m]):
# the tibia in the femur, on the knee-angle domain [-120, 120] deg.
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

# The patella in the tibia, on the knee-angle domain [-120, 10] deg.
PATELLA_CONTROL_FRAMES = [
    (0.309105, 0.019459, -0.024572),
    (0.306824, 0.023835, -0.023992),
    (0.306706, 0.031712, -0.022439),
    (0.308396, 0.040558, -0.022450),
    (0.292224, 0.048174, -0.023697),
    (0.263111, 0.053684, -0.024779),
    (0.033858, 0.056345, -0.025799),
    (-0.284247, 0.055639, -0.025418),
]


def spline_joint(frames, theta_min_deg, theta_max_deg):
    """Planar spline joint whose parameter is the knee angle in degrees."""
    control_frames = [
        pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([tx, ty, 0.0]))
        for (yaw, tx, ty) in frames
    ]
    return (
        pin.JointModelSplineBuilder()
        # degree of the splines, degree 3 means acceleration is continuous.
        .withDegree(3)
        # list of SE3 transforms
        .withControlFrameVector(control_frames)
        # Map knee flexion angle (degrees) to spline parameter q in [0, 1].
        .withOpenUniformKnots(theta_min_deg, theta_max_deg)
        .build()
    )


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

    def add(self, body_pose):
        self._trail.append(self._world_to_femur.act(body_pose.act(self._point)))
        pts = np.asarray(self._trail, dtype=np.float32).T
        self._node.set_object(mg.Points(mg.PointsGeometry(pts), self._material))


def main():
    # Knee-angle domains of the fitted splines (in degree).
    knee = spline_joint(CONTROL_FRAMES, -120.0, 120.0)
    patella = spline_joint(PATELLA_CONTROL_FRAMES, -120.0, 10.0)

    model = pin.Model()
    joint_id = model.addJoint(0, knee, RECENTER, "knee")
    patella_id = model.addJoint(joint_id, patella, pin.SE3.Identity(), "patella")
    # The patella follows the knee: q_patella = 1 * q_knee + 0, no DOF of its own.
    model = pin.transformJointIntoMimic(model, joint_id, patella_id, 1.0, 0.0)
    print(f"nq = {model.nq}, patella joint: {model.joints[patella_id].shortname()}")

    # visual only code
    try:
        visual_model = pin.GeometryModel()
        visual_model.addGeometryObject(
            mesh_object("femur", 0, MESH_DIR / "femur_r.stl", FEMUR_SCALE, RECENTER)
        )
        visual_model.addGeometryObject(
            mesh_object(
                "tibia",
                joint_id,
                MESH_DIR / "tibia_r.stl",
                TIBIA_SCALE,
                pin.SE3.Identity(),
            )
        )
        visual_model.addGeometryObject(
            mesh_object(
                "patella",
                patella_id,
                MESH_DIR / "patella_r.stl",
                PATELLA_SCALE,
                pin.SE3.Identity(),
            )
        )

        viz = MeshcatVisualizer(model, None, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()

    except ImportError as e:
        print("Error while initializing the viewer.")
        print(e)
        return

    # Trace tibia-frame points and the patella origin, and show the tibia frame as a
    # triad, updated every step.
    plateau = np.array([0.03100708, -0.03819558, -0.00347008])
    tracers = [
        (joint_id, PointTracer(viz, "origin", color=0xFF3030)),
        (joint_id, PointTracer(viz, "plateau", plateau, color=0x3060FF)),
        (patella_id, PointTracer(viz, "patella", color=0x30B040)),
    ]
    tibia_frame = viz.viewer["tibia_frame"]
    tibia_frame.set_object(mg.triad(0.08))

    # Sweep the physical knee angle (+10 deg hyperextension to -120 deg flexion),
    angles = np.concatenate([angles, angles[::-1]])
    for _ in range(5):
        for theta_deg in angles:
            viz.display(np.array([theta_deg]))
            for body_id, tracer in tracers:
                tracer.add(viz.data.oMi[body_id])
            tibia_frame.set_transform(viz.data.oMi[joint_id].homogeneous)
            time.sleep(0.015)


if __name__ == "__main__":
    main()
