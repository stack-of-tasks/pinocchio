import os
from pathlib import Path
from sys import argv

import pinocchio

model_path = Path(
    Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR")) if len(argv) < 2 else argv[1]
)
mesh_dir = model_path.parent.parent
urdf_model_path = model_path / "ur_description/urdf/ur5_robot.urdf"

# Load the urdf model
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)
print("model name:", model.name)

# Create data required by the algorithms
data, collision_data, visual_data = pinocchio.createDatas(
    model, collision_model, visual_model
)

# Sample a random configuration
q = pinocchio.randomConfiguration(model)
print(f"q: {q.T}")

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Update Geometry models
pinocchio.updateGeometryPlacements(model, data, collision_model, collision_data)
pinocchio.updateGeometryPlacements(model, data, visual_model, visual_data)

# Print out the placement of each joint of the kinematic tree
print("\nJoint placements:")
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

# Print out the placement of each collision geometry object
print("\nCollision object placements:")
for k, oMg in enumerate(collision_data.oMg):
    print("{:d} : {: .2f} {: .2f} {: .2f}".format(k, *oMg.translation.T.flat))

# Print out the placement of each visual geometry object
print("\nVisual object placements:")
for k, oMg in enumerate(visual_data.oMg):
    print("{:d} : {: .2f} {: .2f} {: .2f}".format(k, *oMg.translation.T.flat))
