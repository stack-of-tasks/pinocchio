import unittest
import pinocchio as se3
import os

class TestGeometryObjectUrdfBindings(unittest.TestCase):

    def test_load(self):
        current_file =  os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.abspath(os.path.join(current_file, '../../models/romeo'))
        model_path = os.path.abspath(os.path.join(model_dir, 'romeo_description/urdf/romeo.urdf'))
        expected_mesh_path = os.path.join(model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')

        hint_list = [model_dir, "wrong/hint"]
        model = se3.buildModelFromUrdf(model_path, se3.JointModelFreeFlyer())
        collision_model = se3.buildGeomFromUrdf(model, model_path, se3.utils.fromListToVectorOfString(hint_list), se3.GeometryType.COLLISION)

        col = collision_model.geometryObjects[1]
        self.assertTrue(col.meshPath == expected_mesh_path)

if __name__ == '__main__':
    unittest.main()
