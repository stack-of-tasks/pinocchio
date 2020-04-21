import pinocchio
from sys import argv
from os.path import dirname, join, abspath

# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

model_path = join(pinocchio_model_dir,"others/robots") if len(argv)<2 else argv[1]
mesh_dir = model_path
urdf_model_path = join(model_path,"ur_description/urdf/ur5_robot.urdf")

# Load the urdf model
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_model_path, mesh_dir)
print('model name: ' + model.name)

# Create data required by the algorithms
data, collision_data, visual_data  = pinocchio.createDatas(model, collision_model, visual_model)

# Sample a random configuration
q        = pinocchio.randomConfiguration(model)
print('q: %s' % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)

# Update Geometry models
pinocchio.updateGeometryPlacements(model, data, collision_model, collision_data)
pinocchio.updateGeometryPlacements(model, data, visual_model, visual_data)

# Print out the placement of each joint of the kinematic tree
print("\nJoint placements:")
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))

# Print out the placement of each collision geometry object
print("\nCollision object placements:")
for k, oMg in enumerate(collision_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))

# Print out the placement of each visual geometry object
print("\nVisual object placements:")
for k, oMg in enumerate(visual_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))
