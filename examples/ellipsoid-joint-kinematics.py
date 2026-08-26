"""
Ellipsoid Joint Kinematics Example

The joint has 3 DOF corresponding to 3 rotation angles that determine both:
- The position on the ellipsoid surface
- The orientation of the body

Note: the joint is not purely normal to the ellipsoid surface;
    it translates along a ellipsoid, but the rotational part is "normal" to a sphere.
"""

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


def create_ellipsoid_robot(radius_x=0.5, radius_y=0.3, radius_z=0.2):
    """
    Create a simple robot with an ellipsoid joint.

    Parameters
    ----------
    radius_x : float
        Ellipsoid radius along x-axis
    radius_y : float
        Ellipsoid radius along y-axis
    radius_z : float
        Ellipsoid radius along z-axis

    Returns
    -------
    model : pin.Model
        The robot model
    """
    model = pin.Model()

    # Create ellipsoid joint
    joint_name = "ellipsoid"
    joint_id = model.addJoint(
        0,  # parent joint id (0 = universe)
        pin.JointModelEllipsoid(radius_x, radius_y, radius_z),
        pin.SE3.Identity(),
        joint_name,
    )

    # Add a body with some inertia
    mass = 1.0
    lever = np.array([0.0, 0.0, 0.1])
    inertia = pin.Inertia(mass, lever, np.diag([0.01, 0.01, 0.01]))

    body_placement = pin.SE3.Identity()
    body_placement.translation = np.array([0.0, 0.0, 0.2])

    model.appendBodyToJoint(joint_id, inertia, body_placement)

    return model


def create_composite_model():
    """Create a model with a composite joint (6 DOF: Tx, Ty, Tz, Rx, Ry, Rz)."""
    model = pin.Model()

    # Create composite joint with 3 prismatic + 3 revolute
    jcomposite = pin.JointModelComposite()
    jcomposite.addJoint(pin.JointModelPX())
    jcomposite.addJoint(pin.JointModelPY())
    jcomposite.addJoint(pin.JointModelPZ())
    jcomposite.addJoint(pin.JointModelRX())
    jcomposite.addJoint(pin.JointModelRY())
    jcomposite.addJoint(pin.JointModelRZ())

    joint_id = model.addJoint(0, jcomposite, pin.SE3.Identity(), "composite")

    body_placement = pin.SE3.Identity()
    body_placement.translation = np.array([0.0, 0.0, 0.2])

    # Add a body with some inertia
    mass = 1.0
    lever = np.array([0.0, 0.0, 0.1])
    inertia = pin.Inertia(mass, lever, np.diag([0.01, 0.01, 0.01]))

    model.appendBodyToJoint(
        joint_id,
        inertia,
        body_placement,
    )

    return model


def compute_kinematics_example():
    """Demonstrate forward kinematics with the ellipsoid joint."""
    print("=" * 60)
    print("ELLIPSOID JOINT KINEMATICS EXAMPLE")
    print("=" * 60)

    # Create model with ellipsoid radii
    radius_x, radius_y, radius_z = 0.5, 0.3, 0.2
    model = create_ellipsoid_robot(radius_x, radius_y, radius_z)
    data = model.createData()

    print("\nEllipsoid parameters:")
    print(f"  radius_x (x-axis): {radius_x}")
    print(f"  radius_y (y-axis): {radius_y}")
    print(f"  radius_z (z-axis): {radius_z}")

    # Test different configurations
    configs = [
        ("Zero configuration", np.array([0.0, 0.0, 0.0])),
        ("Rotation around x", np.array([np.pi / 4, 0.0, 0.0])),
        ("Rotation around y", np.array([0.0, np.pi / 4, 0.0])),
        ("Rotation around z", np.array([0.0, 0.0, np.pi / 4])),
        ("Combined rotations", np.array([np.pi / 6, np.pi / 4, np.pi / 3])),
    ]

    for name, q in configs:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        placement = data.oMi[1]
        position = placement.translation

        print(f"\n{name}:")
        print(f"  q = {q}")
        print(f"  Position on ellipsoid: {position}")
        print(f"  Distance from origin: {np.linalg.norm(position):.4f}")

        # Verify point is on ellipsoid surface
        normalized_pos = position / np.array([radius_x, radius_y, radius_z])

        # checking the quadratic form of the ellipsoid equation
        on_surface = np.abs(np.linalg.norm(normalized_pos) - 1.0) < 1e-10
        print(f"  On ellipsoid surface: {on_surface}")


def compute_dynamics_example():
    """Demonstrate dynamics computation with the ellipsoid joint."""
    print("\n" + "=" * 60)
    print("ELLIPSOID JOINT DYNAMICS EXAMPLE")
    print("=" * 60)

    # Create model
    model = create_ellipsoid_robot(0.5, 0.3, 0.2)
    data = model.createData()

    # Configuration, velocity, and acceleration
    q = np.array([np.pi / 6, np.pi / 4, np.pi / 3])
    v = np.array([0.1, 0.2, 0.3])
    a = np.array([0.01, 0.02, 0.03])

    pin.forwardKinematics(model, data, q, v, a)

    print(f"\nConfiguration: q = {q}")
    print(f"Velocity:      v = {v}")
    print(f"Acceleration:  a = {a}")

    # Compute dynamics with RNEA (Recursive Newton-Euler Algorithm)
    # Important Note:
    #   the generalized forces dimension matches the number of DOFs
    #   but they are not pure torques due to the joint's nature and because the last
    #   The last step of rnea is tau = S.T * f
    #   where S is the joint motion subspace, and f the spatial force of the joint
    #   So the resulting tau vector contains three generalized forces that
    #   include contributions from both rotational and translational effects.
    tau = pin.rnea(model, data, q, v, a)
    print(f"\nRequired torques (RNEA): τ = {tau}")

    # For example tau[0] is a combination of
    # S_11(q)* f_x + S_21(q)* f_y + S_31(q)* f_z
    #   + S_41(q)* τ_x + S_51(q)* τ_y + S_61(q)* τ_z

    # Create composite model for comparison of joint spatial forces `data.f[1]`
    composite_model = create_composite_model()
    composite_data = composite_model.createData()

    q_composite = np.zeros(6)
    q_composite[3:] = q  # Set only rotational part
    q_composite[:3] = data.oMi[
        1
    ].translation  # Set translation to current position on ellipsoid

    v_composite = np.zeros(6)
    v_composite[3:] = v
    v_composite[:3] = np.array(data.v[1])[:3]  # Set linear velocity part

    a_composite = np.zeros(6)
    a_composite[3:] = a
    a_composite[:3] = np.array(data.a[1])[:3]  # Set linear acceleration part

    tau_composite = pin.rnea(
        composite_model, composite_data, q_composite, v_composite, a_composite
    )
    print(f"\nComposite joint required torques (RNEA): τ = {tau_composite}")
    print("\nComparison of spatial forces at the joint:")
    print(f"  - Ellipsoid joint spatial force: f = {data.f[1]} ")
    print(f"  - Composite joint spatial force: f = {composite_data.f[1]} ")


def visualize_ellipsoid_motion():
    """Visualize the ellipsoid joint motion in MeshCat."""

    if not pin.WITH_COLLISION:
        print("\nVisualization skipped (coal not available)")
        print("Install with: conda install -c conda-forge coal")
        print("Then recompile Pinocchio with -DBUILD_WITH_COLLISION_SUPPORT=ON")
        return

    print("\n" + "=" * 60)
    print("ELLIPSOID JOINT VISUALIZATION")
    print("=" * 60)

    # Create model
    radius_x, radius_y, radius_z = 0.5, 0.3, 0.2
    model = create_ellipsoid_robot(radius_x, radius_y, radius_z)

    # Add visual geometry
    geom_model = pin.GeometryModel()

    # 1. Add the ELLIPSOID SURFACE as a visual object
    ellipsoid_shape = pin.coal.Ellipsoid(radius_x, radius_y, radius_z)
    ellipsoid_geom = pin.GeometryObject(
        "ellipsoid_surface",
        0,  # Universe frame
        pin.SE3.Identity(),
        ellipsoid_shape,
    )
    ellipsoid_geom.meshColor = np.array([0.8, 0.8, 0.8, 0.3])  # Semi-transparent gray
    geom_model.addGeometryObject(ellipsoid_geom)

    # 2. Add a small sphere to show the contact point on the ellipsoid
    contact_sphere = pin.coal.Sphere(0.03)
    contact_geom = pin.GeometryObject(
        "contact_point",
        model.getJointId("ellipsoid"),
        pin.SE3.Identity(),  # At the joint frame (on the ellipsoid surface)
        contact_sphere,
    )
    contact_geom.meshColor = np.array([1.0, 0.0, 0.0, 1.0])  # Red
    geom_model.addGeometryObject(contact_geom)

    # 3. Add a box to visualize the body orientation
    box = pin.coal.Box(0.1, 0.1, 0.3)
    box_geom = pin.GeometryObject(
        "body_visual",
        model.getJointId("ellipsoid"),
        pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.15])),
        box,
    )
    box_geom.meshColor = np.array([0.2, 0.6, 1.0, 1.0])  # Blue
    geom_model.addGeometryObject(box_geom)

    # Initialize visualizer
    try:
        viz = MeshcatVisualizer(model, geom_model, geom_model)
        viz.initViewer(open=True)
        viz.loadViewerModel()
    except ImportError as error:
        print(error)
        print("\nVisualization skipped (MeshcatVisualizer not available)")
        return
    print("\n✓ Visualization initialized!")
    print("  - Gray ellipsoid: The constraint surface")
    print("  - Red sphere: Contact point on the surface")
    print("  - Blue box: The rigid body attached to the joint")
    print("\nAnimating ellipsoid joint motion...")
    print("Open http://127.0.0.1:7000/static/ in your browser to see the animation.")

    # Animate through different configurations
    import time

    t = 0.0
    dt = 0.02

    for i in range(500):
        # Create a smooth trajectory on the ellipsoid
        q = np.array(
            [
                0.8 * np.sin(1.0 * t),
                0.6 * np.sin(1.2 * t + 1.0),
                0.4 * np.sin(0.8 * t + 0.5),
            ]
        )

        viz.display(q)
        time.sleep(dt)
        t += dt

        if i % 50 == 0:
            print(
                f"  Frame {i}/500 - Configuration: \n"
                f"    [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}]"
            )

    # create extra motion that slides along the principal axes q0
    for angle in np.linspace(0, 2 * np.pi, 100):
        q = np.array([0.5 * np.cos(angle), 0.0, 0.0])
        viz.display(q)
        time.sleep(dt)

    # q1 axis
    for angle in np.linspace(0, 2 * np.pi, 100):
        q = np.array([0.0, 0.3 * np.cos(angle), 0.0])
        viz.display(q)
        time.sleep(dt)

    # q2 axis
    for angle in np.linspace(0, 2 * np.pi, 100):
        q = np.array([0.0, 0.0, 3.14 * np.cos(angle)])
        viz.display(q)
        time.sleep(dt)

    print("\n✓ Animation completed!")


def main():
    """Run all examples."""
    compute_kinematics_example()
    compute_dynamics_example()
    visualize_ellipsoid_motion()

    print("\n" + "=" * 60)
    print("Examples completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
