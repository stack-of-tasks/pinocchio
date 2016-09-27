import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand
from pinocchio.robot_wrapper import RobotWrapper
import os

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestGeometryObjectBindings(unittest.TestCase):

    v3zero = zero(3)
    v6zero = zero(6)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m6zero = zero([6,6])
    m3ones = eye(3)
    m4ones = eye(4)


    current_file =  os.path.dirname(os.path.abspath(__file__))
    pinocchio_models_dir = os.path.abspath(os.path.join(current_file, '../models/romeo'))
    romeo_model_path = os.path.abspath(pinocchio_models_dir, '/urdf/romeo.urdf')
    hint_list = [pinocchio_models_dir, "wrong/hint"] # hint list
    robot = RobotWrapper(romeo_model_path, hint_list, se3.JointModelFreeFlyer())

    def test_name_get_set(self):
        col = self.robot.collision_model.geometryObjects[1]
        self.assertTrue(col.name == 'LHipPitchLink_0')
        col.name = 'new_collision_name'
        self.assertTrue(col.name == 'new_collision_name')

    def test_parent_get_set(self):
        col = self.robot.collision_model.geometryObjects[1]
        self.assertTrue(col.parentJoint == 4)
        col.parentJoint = 5
        self.assertTrue(col.parentJoint == 5)

    def test_placement_get_set(self):
        m = se3.SE3(self.m3ones, self.v3zero)
        new_m = se3.SE3(rand([3,3]), rand(3))
        col = self.robot.collision_model.geometryObjects[1]
        self.assertTrue(np.allclose(col.placement.homogeneous,m.homogeneous))
        col.placement = new_m
        self.assertTrue(np.allclose(col.placement.homogeneous , new_m.homogeneous))

    def test_meshpath_get(self):
        expected_mesh_path = os.path.join(self.pinocchio_models_dir,'meshes/romeo/collision/LHipPitch.dae')
        col = self.robot.collision_model.geometryObjects[1]
        self.assertTrue(col.meshPath == expected_mesh_path)

if __name__ == '__main__':
    unittest.main()
