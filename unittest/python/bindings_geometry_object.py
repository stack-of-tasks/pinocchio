import unittest
import pinocchio as pin
import numpy as np

@unittest.skipUnless(pin.WITH_HPP_FCL,"Needs HPP-FCL")
class TestGeometryObjectBindings(unittest.TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoid()
        self.collision_model = pin.buildSampleGeometryModelHumanoid(self.model)

    def test_name_get_set(self):
        col = self.collision_model.geometryObjects[0]
        self.assertTrue(col.name == 'rleg_shoulder_object')
        col.name = 'new_collision_name'
        self.assertTrue(col.name == 'new_collision_name')

    def test_parent_get_set(self):
        col = self.collision_model.geometryObjects[0]
        self.assertTrue(col.parentJoint == 2)
        col.parentJoint = 3
        self.assertTrue(col.parentJoint == 3)

    def test_placement_get_set(self):
        m = pin.SE3.Identity()
        new_m = pin.SE3.Random()
        col = self.collision_model.geometryObjects[0]
        self.assertTrue(np.allclose(col.placement.homogeneous,m.homogeneous))
        col.placement = new_m
        self.assertTrue(np.allclose(col.placement.homogeneous , new_m.homogeneous))

    def test_meshpath_get(self):
        col = self.collision_model.geometryObjects[0]
        self.assertTrue(col.meshPath is not None)

    def test_scale(self):
        scale = np.array([1.,2.,3.])
        pin.setGeometryMeshScales(self.collision_model,scale)
        for obj in self.collision_model.geometryObjects:
            self.assertTrue(obj.meshScale[0] == scale[0])
            self.assertTrue(obj.meshScale[1] == scale[1])
            self.assertTrue(obj.meshScale[2] == scale[2])

    def test_scalar_scale(self):
        scale = 2.
        vec = np.array([scale]*3)
        pin.setGeometryMeshScales(self.collision_model,scale)
        for obj in self.collision_model.geometryObjects:
            self.assertTrue(np.allclose(obj.meshScale, vec))

    def test_create_data(self):
        collision_data = self.collision_model.createData()
        self.assertEqual(len(collision_data.oMg), self.collision_model.ngeoms)

    def test_create_datas(self):
        collision_data = self.collision_model.createData()

        self.assertEqual(len(collision_data.oMg), self.collision_model.ngeoms)

        data_2, collision_data_2 = pin.createDatas(self.model, self.collision_model)
        self.assertTrue(self.model.check(data_2))
        self.assertEqual(len(collision_data_2.oMg), self.collision_model.ngeoms)

    def test_copy(self):
        collision_model_copy = self.collision_model.copy()
        self.assertEqual(self.collision_model.ngeoms,collision_model_copy.ngeoms)

        collision_data = self.collision_model.createData()
        collision_data_copy = collision_data.copy()
        self.assertEqual(len(collision_data.oMg),len(collision_data_copy.oMg))

if __name__ == '__main__':
    unittest.main()
