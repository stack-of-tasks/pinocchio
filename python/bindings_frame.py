import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand
from pinocchio.robot_wrapper import RobotWrapper

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestFrameBindings(unittest.TestCase):

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
        f = self.robot.model.operational_frames[1]
        self.assertTrue(f.name == 'r_sole_joint')
        f.name = 'new_sole_joint'
        self.assertTrue(f.name == 'new_sole_joint')

    def test_parent_get_set(self):
        f = self.robot.model.operational_frames[1]
        self.assertTrue(f.parent == 13)
        f.parent = 5
        self.assertTrue(f.parent == 5)

    def test_placement_get_set(self):
        m = se3.SE3(self.m3ones, np.array([0,0,-0.0684],np.double))
        new_m = se3.SE3(rand([3,3]), rand(3))
        f = self.robot.model.operational_frames[1]
        self.assertTrue(np.allclose(f.placement.homogeneous,m.homogeneous))
        f.placement = new_m
        self.assertTrue(np.allclose(f.placement.homogeneous , new_m.homogeneous))


if __name__ == '__main__':
    unittest.main()
