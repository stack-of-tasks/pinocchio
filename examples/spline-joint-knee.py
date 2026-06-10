"""Knee spline joint driven by its physical flexion angle.

This example uses the SE(3) spline joint (``pin.JointModelSpline`` /
``pin.JointModelSplineBuilder``) to reproduce experimental knee kinematics: the
joint constrains its child (tibia) frame to a cumulative B-spline on SE(3)
defined by a list of control frames, a knot vector and a degree (Lee et al.,
"Spline Joints for Multibody Dynamics").

The 8 control frames below were fit offline (Gauss-Newton, degree=3, n_ctrl=8)
to a planar OpenSim knee, where the rotation about Z and the tx/ty translations
are driven by ``knee_angle_r``. The knee angle spans [-120 deg, +120 deg] and
the spline parameter q is its affine normalization, so the joint is driven
directly by a physical angle (``q_from_knee_angle``).

Running it opens Meshcat and flexes a femur + tibia, showing the tibia frame and
the dotted trajectories of tibia points (e.g. the origin and the tibial plateau).
"""

import time
from pathlib import Path

import coal
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Knee-angle domain of the fitted spline (used by q_from_knee_angle).
THETA_MIN_DEG, THETA_MAX_DEG = -120.0, 120.0

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


def q_from_knee_angle(theta_deg):
    """Affine map from knee flexion angle (degrees) to spline parameter q in [0, 1]."""
    return (theta_deg - THETA_MIN_DEG) / (THETA_MAX_DEG - THETA_MIN_DEG)


def build_model():
    """Build a model with a single spline knee joint. Returns (model, joint_id)."""
    control_frames = [
        pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([tx, ty, 0.0]))
        for (yaw, tx, ty) in CONTROL_FRAMES
    ]
    knee = (
        pin.JointModelSplineBuilder()
        .withDegree(3)
        .withControlFrameVector(control_frames)
        .withOpenUniformKnots(0.0, 1.0)
        .build()
    )
    model = pin.Model()
    joint_id = model.addJoint(0, knee, RECENTER, "knee")
    return model, joint_id


def mesh_object(name, parent_joint, stl, scale, placement):
    """Build a mesh GeometryObject fixed to ``parent_joint`` at ``placement``."""
    mesh_scale = scale * np.ones(3)
    geom = coal.MeshLoader().load(str(stl), mesh_scale)
    obj = pin.GeometryObject(name, parent_joint, placement, geom, str(stl), mesh_scale)
    obj.meshColor = BONE_COLOR
    return obj


def build_visual_model(joint_id):
    """Femur fixed in the world (condyle at origin) and tibia on the knee joint."""
    visual_model = pin.GeometryModel()
    visual_model.addGeometryObject(
        mesh_object("femur", 0, MESH_DIR / "femur_r.stl", FEMUR_SCALE, RECENTER)
    )
    visual_model.addGeometryObject(
        mesh_object(
            "tibia", joint_id, MESH_DIR / "tibia_r.stl", TIBIA_SCALE, pin.SE3.Identity()
        )
    )
    return visual_model


def tibia_landmarks(stl, scale):
    """Return (plateau, ankle) points in the tibia frame, read from the mesh.

    ``plateau`` is the top of the tibial plateau closest to the sagittal (xy)
    plane; ``ankle`` is the most distal vertex.
    """
    mesh = coal.MeshLoader().load(str(stl), scale * np.ones(3))
    V = np.asarray(mesh.vertices())
    top = V[V[:, 1] >= V[:, 1].max() - 0.02]  # within 2 cm of the top.
    plateau = top[np.argmin(np.abs(top[:, 2]))]  # closest to the xy plane.
    ankle = V[np.argmin(V[:, 1])]  # most distal vertex.
    return plateau, ankle


class PointTracer:
    """Trace a fixed tibia-frame point in Meshcat, keeping the dotted trail.

    Each :meth:`add` appends one dot at the point's current world position;
    nothing is erased. Dots ride with the femur (drawn under a node at
    ``RECENTER``), so they sit exactly on the traced point.
    """

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
    model, joint_id = build_model()
    visual_model = build_visual_model(joint_id)

    try:
        viz = MeshcatVisualizer(model, visual_model, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()
    except ImportError as e:
        print("Error while initializing the viewer.")
        print(e)
        return

    # Front sagittal view: look at the xy plane along -z (camera on +z), y up.
    # Use the camera nodes directly (set_cam_pos/target need a newer meshcat).
    viz._node_default_cam.set_transform(np.eye(4))  # target at the knee (origin).
    viz._node_default_cam[viz._rot_cam_key].set_property("position", [0.0, 0.0, 1.6])

    # Trace tibia-frame points (origin glide, tibial plateau) and show the tibia
    # frame itself as a triad, updated every step.
    plateau, _ankle = tibia_landmarks(MESH_DIR / "tibia_r.stl", TIBIA_SCALE)
    tracers = [
        PointTracer(viz, "origin", color=0xFF3030),
        PointTracer(viz, "plateau", plateau, color=0x3060FF),
    ]
    tibia_frame = viz.viewer["tibia_frame"]
    tibia_frame.set_object(mg.triad(0.08))

    # Sweep the physical knee angle (+10 deg hyperextension to -120 deg flexion),
    # back and forth, accumulating the traces.
    angles = np.linspace(10.0, -120.0, 120)
    angles = np.concatenate([angles, angles[::-1]])
    for _ in range(5):
        for theta_deg in angles:
            viz.display(np.array([q_from_knee_angle(theta_deg)]))
            tibia_pose = viz.data.oMi[joint_id]
            for tracer in tracers:
                tracer.add(tibia_pose)
            tibia_frame.set_transform(tibia_pose.homogeneous)
            time.sleep(0.015)


if __name__ == "__main__":
    main()
