\page md_doc_a-features_f-kinematic Kinematics algorithms

## Forward kinematics

Pinocchio implements direct kinematic computations up to the second
order. When a robot configuration is given, a forward pass is performed
to compute the spatial placements of each joint and to store them as
coordinate transformations. If the velocity is given, it also computes
the spatial velocities of each joint (expressed in local frame), and
similarly for accelerations.

## Kinematic Jacobian

The spatial Jacobian of each joint can be easily computed with a single
forward pass, either expressed locally or in the world frame.
