import pinocchio as pin
import numpy as np
import hppfcl as fcl
# Perform the simulation of a four-bar linkages mechanism

height = 0.1
width = 0.01
radius = 0.05

mass_link_A = 10.
length_link_A = 1.
#shape_link_A = fcl.Box(length_link_A,width,height)
shape_link_A = fcl.Capsule(radius,length_link_A)

mass_link_B = 5.
length_link_B = .6
#shape_link_B = fcl.Box(length_link_B,width,height)
shape_link_B = fcl.Capsule(radius,length_link_B)

inertia_link_A = pin.Inertia.FromBox(mass_link_A,length_link_A,width,height)
placement_center_link_A = pin.SE3.Identity()
placement_center_link_A.translation = pin.XAxis * length_link_A / 2.
placement_shape_A = placement_center_link_A.copy()
placement_shape_A.rotation = pin.Quaternion.FromTwoVectors(pin.ZAxis,pin.XAxis).matrix()

inertia_link_B = pin.Inertia.FromBox(mass_link_B,length_link_B,width,height)
placement_center_link_B = pin.SE3.Identity()
placement_center_link_B.translation = pin.XAxis * length_link_B / 2.
placement_shape_B = placement_center_link_B.copy()
placement_shape_B.rotation = pin.Quaternion.FromTwoVectors(pin.ZAxis,pin.XAxis).matrix()

model = pin.Model()
collision_model = pin.GeometryModel()

RED_COLOR = np.array([1.,0.,0.,1.])
WHITE_COLOR = np.array([1.,1.,1.,1.])

base_joint_id = 0
geom_obj0 = pin.GeometryObject("link_A1",base_joint_id,shape_link_A,pin.SE3(pin.Quaternion.FromTwoVectors(pin.ZAxis,pin.XAxis).matrix(),np.zeros((3))))
geom_obj0.meshColor = WHITE_COLOR
collision_model.addGeometryObject(geom_obj0)

joint1_placement = pin.SE3.Identity()
joint1_placement.translation = pin.XAxis * length_link_A/2. 
joint1_id = model.addJoint(base_joint_id,pin.JointModelRY(),joint1_placement,"link_B1")
model.appendBodyToJoint(joint1_id,inertia_link_B,placement_center_link_B)
geom_obj1 = pin.GeometryObject("link_B1",joint1_id,shape_link_B,placement_shape_B)
geom_obj1.meshColor = RED_COLOR
collision_model.addGeometryObject(geom_obj1)

joint2_placement = pin.SE3.Identity()
joint2_placement.translation = pin.XAxis * length_link_B
joint2_id = model.addJoint(joint1_id,pin.JointModelRY(),joint2_placement,"link_A2")
model.appendBodyToJoint(joint2_id,inertia_link_A,placement_center_link_A)
geom_obj2 = pin.GeometryObject("link_A2",joint2_id,shape_link_A,placement_shape_A)
geom_obj2.meshColor = WHITE_COLOR
collision_model.addGeometryObject(geom_obj2)

joint3_placement = pin.SE3.Identity()
joint3_placement.translation = pin.XAxis * length_link_A
joint3_id = model.addJoint(joint2_id,pin.JointModelRY(),joint3_placement,"link_B2")
model.appendBodyToJoint(joint3_id,inertia_link_B,placement_center_link_B)
geom_obj3 = pin.GeometryObject("link_B2",joint3_id,shape_link_B,placement_shape_B)
geom_obj3.meshColor = RED_COLOR
collision_model.addGeometryObject(geom_obj3)


# Parallelogram from sdf

from pinocchio.robot_wrapper import RobotWrapper
robot = RobotWrapper.BuildFromSDF("parallel.sdf")
model_sdf = robot.model

assert(model.nq == model_sdf.nq);
assert(model.nv == model_sdf.nv);
assert(model.njoints == model_sdf.njoints);
assert(model.nbodies == model_sdf.nbodies);
# assert(model.nframes == model_sdf.nframes)
assert(list(model.parents) == list(model_sdf.parents))

for k in range(len(model.children)):
    for j in range(len(model.children[k])):
        assert(model.children[k][j] == model_sdf.children[k][j])

#assert(model.names == model_sdf.names);
for k in range(len(model.subtrees)):
    for j in range(len(model.subtrees[k])):
        assert(model.subtrees[k][j] == model_sdf.subtrees[k][j])

assert(model.gravity == model_sdf.gravity);
#assert(model.name == model_sdf.name);
for k in range(len(model.idx_qs)):
    assert(model.idx_qs[k] == model_sdf.idx_qs[k])
        
assert(list(model.nqs) == list(model_sdf.nqs));
for k in range(len(model.idx_vs)):
    assert(model.idx_vs[k] == model_sdf.idx_vs[k])
assert(list(model.nvs) == list(model_sdf.nvs));
assert(len(model.jointPlacements) == len(model_sdf.jointPlacements))
assert(len(model.inertias) == len(model_sdf.inertias))

for k in range(1, len(model.jointPlacements)):
    assert(model.jointPlacements[k].isApprox(model_sdf.jointPlacements[k]))

for k in range(1, len(model.inertias)):
    assert(model.inertias[k].isApprox(model_sdf.inertias[k]))
