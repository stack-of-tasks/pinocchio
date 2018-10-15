import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand
import os

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

# TODO: remove these lines. Do not use RobotWrapper. Do not use URDF
from pinocchio.robot_wrapper import RobotWrapper
current_file =  os.path.dirname(os.path.abspath(__file__))
romeo_model_dir = os.path.abspath(os.path.join(current_file, '../models/romeo'))
romeo_model_path = os.path.abspath(os.path.join(romeo_model_dir, 'romeo_description/urdf/romeo.urdf'))
hint_list = [romeo_model_dir, "wrong/hint"] # hint list
robot = RobotWrapper(romeo_model_path, hint_list, se3.JointModelFreeFlyer())
expected_mesh_path = os.path.join(romeo_model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')

class TestGeometryObjectBindings(unittest.TestCase):

    v3zero = zero(3)
    v6zero = zero(6)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m6zero = zero([6,6])
    m3ones = eye(3)
    m4ones = eye(4)

    # WARNING: the collision model is the same object is the same for all the tests.
    # This can cause problems if a test expects the collision model to be in a certain way but a previous test has changed it
    # Still, at the moment this is not the case.
    # Loading the URDF and related meshes before each test would make the whole unit test run too slowly
    # 
    # TODO: do not use URDF. Then, build self.collision_model from scratch for each test
    def setUp(self):
        self.collision_model = robot.collision_model
        self.expected_mesh_path = expected_mesh_path

    def test_name_get_set(self):
        col = self.collision_model.geometryObjects[1]
        self.assertTrue(col.name == 'LHipPitchLink_0')
        col.name = 'new_collision_name'
        self.assertTrue(col.name == 'new_collision_name')

    def test_parent_get_set(self):
        col = self.collision_model.geometryObjects[1]
        self.assertTrue(col.parentJoint == 4)
        col.parentJoint = 5
        self.assertTrue(col.parentJoint == 5)

    def test_placement_get_set(self):
        m = se3.SE3(self.m3ones, self.v3zero)
        new_m = se3.SE3(rand([3,3]), rand(3))
        col = self.collision_model.geometryObjects[1]
        self.assertTrue(np.allclose(col.placement.homogeneous,m.homogeneous))
        col.placement = new_m
        self.assertTrue(np.allclose(col.placement.homogeneous , new_m.homogeneous))

    def test_meshpath_get(self):
        col = self.collision_model.geometryObjects[1]
        self.assertTrue(col.meshPath == self.expected_mesh_path)

if __name__ == '__main__':
    unittest.main()
