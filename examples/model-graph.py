import numpy as np
import pinocchio as pin

# This example show how to:
#  - Construct a kinematics chain with ModelGraph API
#  - Bias a joint
#  - Merge two kinematics chain into a kinematics tree
#  - Lock a joint in a particular configuration
#  - Create a Model with a custom root joint

g = pin.graph.ModelGraph()

# Adding bodies to the pin.graph.
# In pinocchio, since bodies are represented as frames,
# there's 2 ways to add a body to the graph :
#   - addBody, helper function
#   - addFrame, where you create the BodyFrame, yourself
g.addBody("body1", pin.Inertia.Identity())
g.addFrame("body2", pin.graph.BodyFrame(pin.Inertia.Identity()))
g.addFrame("body3", pin.graph.BodyFrame(pin.Inertia.Identity()))

# Adding a sensor to the pin.graph.
g.addFrame("sensor1", pin.graph.SensorFrame())

# Now we add the joints between every body/sensor we have in the pin.graph.
pose_b1_to_j1 = pin.SE3.Random()  # pose of joint j1 wrt body1
pose_j1_to_b2 = pin.SE3.Random()  # pose of body2 wrt joint j1

# There are 2 ways to add joints to the graph.
# This helper function, where you specify the basics of a joint only.
g.addJoint(
    "j1",
    pin.graph.JointRevolute(np.array([0.0, 0.0, 1.0])),
    "body1",
    pose_b1_to_j1,
    "body2",
    pose_j1_to_b2,
)

# j2 will be biased by 50cm.
pose_b2_to_j2 = pin.SE3.Random()  # pose of joint j2 wrt body2
pose_j2_to_b3 = pin.SE3.Random()  # pose of body3 wrt joint j2

# To add a joint with more details, such as limits, bias... the
# builder interface is necessary.
g.edgeBuilder().withName("j2").withJointType(
    pin.graph.JointPrismatic(np.array([1, 0, 0]))
).withSourceVertex("body2").withSourcePose(pose_b2_to_j2).withTargetVertex(
    "body3"
).withTargetPose(pose_j2_to_b3).withQref(np.array([0.5])).build()

# sensor1 is a sensor frame so it can only be linked to the others body
# via a fixed joint
pose_b1_s1 = pin.SE3.Random()
pose_s1 = pin.SE3.Random()

g.edgeBuilder().withName("b1_s1").withJointType(
    pin.graph.JointFixed()
).withSourceVertex("body1").withSourcePose(pose_b1_s1).withTargetVertex(
    "sensor1"
).withTargetPose(pose_s1).build()

# Now we can choose which body will be our root its position, and build the model
kinematics_chain_from_body1 = pin.graph.buildModel(g, "body1", pin.SE3.Identity())
print("Kinematics chain from body1:")
print(kinematics_chain_from_body1)

# To merge two model, we can create a new ModelGraph and merge it to the first one.
# To simplify the process, we will append g to g.
# Since all joints and frames should have an unique name,
# we will use pin.graph.prefixNames function.
g1 = pin.graph.prefixNames(g, "g1/")
g2 = pin.graph.prefixNames(g, "g2/")

# Then we will attach g2/body3 to g1/body2 with a spherical joint.
# Equivalent to appendModel (deprecated now)
g1_g2_merged = pin.graph.merge(
    g1, g2, "g1/body2", "g2/body3", pin.SE3.Random(), pin.graph.JointSpherical()
)

# We can then create our model with any body as a root.
kinematics_tree_from_g1_body1 = pin.graph.buildModel(
    g1_g2_merged, "g1/body1", pin.SE3.Identity()
)
print("Kinematics tree from g1/body1:")
print(kinematics_tree_from_g1_body1)

# To lock a joints we only have to provide his name and his reference configuration.
# We will lock g1/j1 at 0.3 rad.
# Equivalent to buildReducedModel (deprecated now)
g1_g2_merged_locked = pin.graph.lockJoints(g1_g2_merged, ["g1/j1"], [np.array([0.3])])

# We can then create the locked model.
# We will use g2/body2 as root with a FreeFlyer joint.
kinematics_tree_from_g2_body2 = pin.graph.buildModel(
    g1_g2_merged_locked, "g2/body2", pin.SE3.Identity(), pin.graph.JointFreeFlyer()
)
print("Kinematics tree with g1/j1 locked and FreeFlyer from g2/body2:")
print(kinematics_tree_from_g2_body2)
