import unittest
import pinocchio as pin
pin.switchToNumpyMatrix()
import os

@unittest.skipUnless(pin.WITH_URDFDOM_SUPPORT(),"Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):

    def setUp(self):
        self.current_file =  os.path.dirname(os.path.abspath(__file__))
        self.model_dir = os.path.abspath(os.path.join(self.current_file, '../../models/romeo'))
        self.model_path = os.path.abspath(os.path.join(self.model_dir, 'romeo_description/urdf/romeo.urdf'))

    def test_load(self):
        hint_list = [self.model_dir, "wrong/hint"]
        expected_mesh_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')

        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_list, pin.GeometryType.COLLISION)

        col = collision_model.geometryObjects[1]
        self.assertEqual(col.meshPath, expected_mesh_path)

    def test_multi_load(self):
        hint_list = [self.model_dir, "wrong/hint"]
        expected_collision_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')
        expected_visual_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/visual/LHipPitch.dae')

        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

        collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_list, pin.GeometryType.COLLISION)
        col = collision_model.geometryObjects[1]
        self.assertEqual(col.meshPath, expected_collision_path)

        visual_model = pin.buildGeomFromUrdf(model, self.model_path, hint_list, pin.GeometryType.VISUAL)
        vis = visual_model.geometryObjects[1]
        self.assertEqual(vis.meshPath, expected_visual_path)

        model_2, collision_model_2, visual_model_2 = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer())

        self.assertEqual(model,model_2)

        col_2 = collision_model_2.geometryObjects[1]
        self.assertEqual(col_2.meshPath, expected_collision_path)

        vis_2 = visual_model_2.geometryObjects[1]
        self.assertEqual(vis_2.meshPath, expected_visual_path)

        model_c, collision_model_c = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer(), geometry_types=pin.GeometryType.COLLISION)

        self.assertEqual(model,model_c)

        col_c = collision_model_c.geometryObjects[1]
        self.assertEqual(col_c.meshPath, expected_collision_path)

        model_v, visual_model_v = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer(), geometry_types=pin.GeometryType.VISUAL)

        self.assertEqual(model,model_v)

        vis_v = visual_model_v.geometryObjects[1]
        self.assertEqual(vis_v.meshPath, expected_visual_path)

if __name__ == '__main__':
    unittest.main()
