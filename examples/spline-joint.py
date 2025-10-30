import pinocchio as pin
import numpy as np
import hppfcl
import time
from pinocchio.visualize import MeshcatVisualizer

def generate_random_se3_trajectory(num_steps, radius, num_revolutions, height):
    """
    Generates a random trajectory in SE(3) with rotating orientation.

    Args:
      num_keyframes: The number of random keyframes to generate.
      num_steps_per_segment: The number of intermediate steps between each keyframe.

    Returns:
      A list of pinocchio.SE3 objects representing the trajectory.
    """
    trajectory = []
    for i in range(num_steps):
        # 1. Parameter to track progress along the helix (from 0.0 to 1.0)
        alpha = float(i) / num_steps

        # 2. Define the translational part (the helical path)
        # The angle determines the position on the XY plane
        angle = alpha * num_revolutions * 2 * np.pi
        
        # Calculate the x, y, z coordinates for the helix
        translation = np.array([
            radius * np.cos(angle),
            radius * np.sin(angle),
            alpha * height
        ])

        # 3. Define the rotational part (a new random orientation at each step)
        # pin.SE3.Random().rotation generates a random 3x3 rotation matrix
        random_rotation = pin.SE3.Random().rotation

        # 4. Combine the translation and random rotation into a single SE(3) pose
        pose = pin.SE3(random_rotation, translation)
        trajectory.append(pose)

    return trajectory
# --- Visualization Setup ---

# Generate the random trajectory
# Parameters for the helical trajectory
num_steps = 100
radius = 1.0
num_revolutions = 3
height = 2

trajectory = generate_random_se3_trajectory(num_steps, radius, num_revolutions,height)

# Create a Pinocchio model with a single free-flyer joint
model = pin.Model()
joint_id = model.addJoint(0, pin.JointModelSpline(trajectory, 3), pin.SE3.Identity(), "free_flyer")

# Attach a simple visual geometry (a box) to the joint
visual_model = pin.GeometryModel()
box_shape = hppfcl.Box(0.1, 0.2, 0.3)
# The placement of the geometry with respect to the joint frame
geom_placement = pin.SE3.Identity()
geom_obj = pin.GeometryObject("box", joint_id, geom_placement, box_shape)
# Assign a color to the geometry
geom_obj.meshColor = np.array([1.0, 0.5, 0.5, 1.0]) # RGBA
visual_model.addGeometryObject(geom_obj)

# --- Main Execution ---

# Initialize the MeshCat visualizer.
try:
    viz = MeshcatVisualizer(model, visual_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
except ImportError as e:
    print("Error while initializing the viewer. It seems you should install Python meshcat.")
    print(e)
    sys.exit(0)


time.sleep(0.1)

q = pin.neutral(model)

q_vector = np.arange(0, 1, 0.05)
for q in  q_vector:
    # Display the new configuration.
    viz.display(np.array([q]))

    # Delay for visualization
    time.sleep(0.05)
