\page md_doc_a-features_d-model Geometric and collision models

Aside from the kinematic model, Pinocchio defines a geometric model, i.e. the
volumes attached to the kinematic tree. This model can be used for displaying
the robot and computing quantities associated to collisions. Like the kinematic
model, the fixed quantities (placement and shape of the volumes) are stored in
a *GeometricModel* object, while buffers and quantities used by associated
algorithms are defined in an object. The volumes are represented using the FCL
library. Bodies of the robot are attached to each joint, while obstacles of the
environment are defined in the world frame. Collision and distance algorithms
for the kinematic trees are implemented, based on FCL methods.
