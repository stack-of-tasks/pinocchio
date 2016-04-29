import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand
from pinocchio.robot_wrapper import RobotWrapper

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestGeometryObjectBindings(unittest.TestCase):

    v3zero = zero(3)
    v6zero = zero(6)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m6zero = zero([6,6])
    m3ones = eye(3)
    m4ones = eye(4)

    hint_list = ["/local/fvalenza/devel/src/pinocchio/models", "wrong/hint"] # hint list
    robot = RobotWrapper("/local/fvalenza/devel/src/pinocchio/models/romeo.urdf", hint_list, se3.JointModelFreeFlyer())

    def test_name_get_set(self):
        col = self.robot.geometry_model.collision_objects[1]
        self.assertTrue(col.name == '') # See in parser why name is empty
        col.name = 'new_collision_name'
        self.assertTrue(col.name == 'new_collision_name')

    def test_parent_get_set(self):
        col = self.robot.geometry_model.collision_objects[1]
        self.assertTrue(col.parent == 3)
        col.parent = 5
        self.assertTrue(col.parent == 5)

    def test_placement_get_set(self):
        m = se3.SE3(self.m3ones, self.v3zero)
        new_m = se3.SE3(rand([3,3]), rand(3))
        col = self.robot.geometry_model.collision_objects[1]
        self.assertTrue(np.allclose(col.placement.homogeneous,m.homogeneous))
        col.placement = new_m
        self.assertTrue(np.allclose(col.placement.homogeneous , new_m.homogeneous))

    def test_meshpath_get(self):
        col = self.robot.geometry_model.collision_objects[1]
        self.assertTrue(col.mesh_path == '/local/fvalenza/devel/src/pinocchio/models/meshes/romeo/collision/LHipPitch.dae')

if __name__ == '__main__':
    unittest.main()
