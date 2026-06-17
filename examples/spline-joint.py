import time

import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


class PointTracer:
    """Trace a fixed point in Meshcat, keeping the dotted trail."""

    def __init__(self, viz):
        self._material = mg.PointsMaterial(size=0.004, color=0xFF3030)
        self._trail = []
        self._node = viz.viewer["solid_pose"]

    def add(self, solid_pose):
        self._trail.append(solid_pose.translation.copy())
        pts = np.asarray(self._trail, dtype=np.float32).T

        line = mg.Line(
            geometry=mg.PointsGeometry(pts),
            material=mg.LineBasicMaterial(color=0xFF3030),
        )
        self._node.set_object(geometry=line)


def main():

    PARABOLA_CONTROL_FRAMES = [
        (-1.000000, 1.000000),
        (-0.888889, 0.777778),
        (-0.666667, 0.407407),
        (-0.333333, 0.074074),
        (0.000000, -0.037037),
        (0.333333, 0.074074),
        (0.666667, 0.407407),
        (0.888889, 0.777778),
        (1.000000, 1.000000),
    ]

    control_frames = [
        pin.SE3(np.eye(3), np.array([0, ty, tz]))
        for (ty, tz) in PARABOLA_CONTROL_FRAMES
    ]

    # Create a Pinocchio model with a single free-flyer joint
    model = pin.Model()
    spline_joint = (
        pin.JointModelSplineBuilder()
        .withDegree(3)
        .withControlFrameVector(control_frames)
        .withOpenUniformKnots(-1, 1)
        .build()
    )
    joint_id = model.addJoint(0, spline_joint, pin.SE3.Identity(), "spline-joint")

    # adding some inertia to drop the solid in the parabola
    box_size = 1.0  # edge length of each cube [m]
    box_mass = 1.0  # mass of each cube [kg]
    box_inertia = pin.Inertia.FromBox(box_mass, box_size, box_size, box_size)
    model.appendBodyToJoint(joint_id, box_inertia, pin.SE3.Identity())

    # visual only code
    try:
        visual_model = pin.GeometryModel()

        viz = MeshcatVisualizer(model, None, visual_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()

    except ImportError as e:
        print("Error while initializing the viewer.")
        print(e)
        return

    tracers = [PointTracer(viz)]
    solid_frame = viz.viewer["solid_frame"]
    solid_frame.set_object(mg.triad(0.2))

    dt = 0.01
    qs, _ = sim_loop(model, dt)

    for theta in qs:
        viz.display(np.array([theta]))
        solid_pose = viz.data.oMi[joint_id]
        for tracer in tracers:
            tracer.add(solid_pose)
        solid_frame.set_transform(solid_pose.homogeneous)
        time.sleep(dt)


def sim_loop(model, dt=0.01, nsteps=800):

    qs = [np.array([1.0])]
    vs = [np.array([0])]
    data = model.createData()
    for i in range(nsteps):
        q = qs[i]
        v = vs[i]
        tau = -1 * v  # a little bit of damping
        a1 = pin.aba(model, data, q, v, tau)
        vnext = v + dt * a1
        qnext = pin.integrate(model, q, dt * vnext)
        qs.append(qnext)
        vs.append(vnext)
    return qs, vs


if __name__ == "__main__":
    main()
