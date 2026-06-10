"""Knee spline joint drives a planar knee joint by its flexion angle.

This example uses the SE(3) spline joint (``pin.JointModelSpline`` /
``pin.JointModelSplineBuilder``) to reproduce experimental knee kinematics. The
joint constrains its child frame to a cumulative B-spline on SE(3) defined by a
list of control frames, a knot vector and a degree
(Lee et al., "Spline Joints for Multibody Dynamics").

The 8 control frames embedded below were produced offline by a Gauss-Newton
regression that fits a ``degree=3, n_ctrl=8`` spline.
The kinematics come from an OpenSim Model leg6dof9musc.osim

  * rotation about Z, driven linearly by ``knee_angle_r``;
  * translations tx / ty, driven by SimmSplines.

The knee angle domain is [-120 deg, +120 deg]. The spline parameter q is the
affine normalization of the knee angle,
q = (theta - theta_min) / (theta_max - theta_min).
"""

import time
from pathlib import Path

import coal
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

THETA_MIN_DEG, THETA_MAX_DEG = -120.0, 120.0  # data domain of the fitted spline.

# Displaying the knee at the center of the origin
KNEE_FROM_HIP_OFFSET = np.array([-0.0046, -0.4644, 0.0])
RECENTER = pin.SE3(np.eye(3), -KNEE_FROM_HIP_OFFSET)


def q_from_knee_angle(theta_deg):
    """Affine map from knee flexion angle (degrees) to spline parameter q in [0, 1]."""
    return (theta_deg - THETA_MIN_DEG) / (THETA_MAX_DEG - THETA_MIN_DEG)


# Control frames from the 2D-knee Gauss-Newton OFFLINE regression (degree=3, n_ctrl=8).
# Each is planar: a rotation about Z by ``yaw`` and a translation [tx, ty, 0].
# (yaw [rad], tx [m], ty [m])
_CONTROL_FRAMES_DATA = [
    (-2.094400, -0.003842, -0.496205),
    (-1.815147, 0.001278, -0.489246),
    (-1.256640, 0.006243, -0.480186),
    (-0.418880, -0.002220, -0.466444),
    (0.418880, -0.008427, -0.463391),
    (1.256640, -0.005740, -0.461613),
    (1.815147, -0.006127, -0.464906),
    (2.094400, -0.006285, -0.464416),
]


def _mesh_geometry_object(name, parent_joint, stl_path, scale, placement):
    mesh_scale = scale * np.ones(3)
    geom = coal.MeshLoader().load(str(stl_path), mesh_scale)
    obj = pin.GeometryObject(
        name, parent_joint, placement, geom, str(stl_path), mesh_scale
    )
    obj.meshColor = np.array([0.96, 0.94, 0.86, 0.7])  # bone ivory.
    return obj


def draw_spline_curve(viz, model, joint_id, n_samples=200):
    """Draw the spline path and its control polygon, expressed in the femur frame.

    The spline describes the tibia's motion relative to the femur, so the curve
    naturally lives in the femur frame. We strip the recenter offset to get the
    raw spline X(q) (and raw control frames), then place the whole drawing under
    one node carrying the femur's placement (``RECENTER``).
    """
    femur_Minv = RECENTER.inverse()  # world -> femur frame.

    data = model.createData()
    curve = np.empty((3, n_samples), dtype=np.float32)
    for i, q in enumerate(np.linspace(0.0, 1.0, n_samples)):
        pin.forwardKinematics(model, data, np.array([q]))
        curve[:, i] = (femur_Minv * data.oMi[joint_id]).translation

    # Control-frame origins (the control polygon), in the femur frame.
    ctrl = np.array(
        [
            pin.SE3(
                pin.rpy.rpyToMatrix(0.0, 0.0, yaw + np.pi/2), np.array([tx, ty, 0.0])
            ).translation
            for (yaw, tx, ty) in _CONTROL_FRAMES_DATA
        ],
        dtype=np.float32,
    ).T

    # The whole drawing is the femur frame, placed at RECENTER in the world.
    node = viz.viewer["femur_frame/spline"]
    node.set_transform(RECENTER.homogeneous)
    node["curve"].set_object(
        mg.Line(mg.PointsGeometry(curve), mg.LineBasicMaterial(color=0xFF3030))
    )
    node["control_points"].set_object(
        mg.Points(
            mg.PointsGeometry(ctrl), mg.PointsMaterial(size=0.001, color=0x3030FF)
        )
    )


def main():
    model = pin.Model()

    knee_control_frames = [
        pin.SE3(pin.rpy.rpyToMatrix(0.0, 0.0, yaw), np.array([tx, ty, 0.0]))
        for (yaw, tx, ty) in _CONTROL_FRAMES_DATA
    ]

    knee = (
        pin.JointModelSplineBuilder()
        .withDegree(3)
        .withControlFrameVector(knee_control_frames)
        .withOpenUniformKnots(0.0, 1.0)
        .build()
    )
    joint_id = model.addJoint(0, knee, RECENTER, "knee")

    # Visualization
    try:
        # Femur + tibia meshes and their OpenSim (subject-specific) scale factors.
        _MESH_DIR = Path(__file__).resolve().parent.parent / "models" / "biomechanics"
        _FEMUR_SCALE = 1.17378
        _TIBIA_SCALE = 1.12373

        visual_model = pin.GeometryModel()
        # Femur: fixed to the universe frame (parent joint 0); condyle at origin.
        visual_model.addGeometryObject(
            _mesh_geometry_object(
                "femur", 0, _MESH_DIR / "femur_r.stl", _FEMUR_SCALE, RECENTER
            )
        )
        # Tibia: rigidly attached to the spline knee joint's child frame
        visual_model.addGeometryObject(
            _mesh_geometry_object(
                "tibia",
                joint_id,
                _MESH_DIR / "tibia_r.stl",
                _TIBIA_SCALE,
                pin.SE3.Identity(),
            )
        )

        viz = MeshcatVisualizer(model, visual_model, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()
    except ImportError as e:
        print("Error while initializing the viewer.")
        print(e)
        return

    draw_spline_curve(viz, model, joint_id)

    time.sleep(0.1)

    # Drive the joint by physical knee flexion angle over the physiological
    # range (slight hyperextension at +10 deg to deep flexion at -120 deg),
    # sweeping forward and back.
    sweep = np.linspace(10.0, -120.0, 120)
    angles = np.concatenate([sweep, sweep[::-1]])
    n_cycles = 3
    for _ in range(n_cycles):
        for theta_deg in angles:
            q = q_from_knee_angle(theta_deg)
            viz.display(np.array([q]))
            time.sleep(0.015)


if __name__ == "__main__":
    main()
