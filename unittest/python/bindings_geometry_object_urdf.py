import unittest
import pinocchio as pin
import os

@unittest.skipUnless(pin.WITH_URDFDOM_SUPPORT(),"Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):

    def test_load(self):
        current_file =  os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.abspath(os.path.join(current_file, '../../models/romeo'))
        model_path = os.path.abspath(os.path.join(model_dir, 'romeo_description/urdf/romeo.urdf'))
        expected_mesh_path = os.path.join(model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')

        hint_list = [model_dir, "wrong/hint"]
        model = pin.buildModelFromUrdf(model_path, pin.JointModelFreeFlyer())
        collision_model = pin.buildGeomFromUrdf(model, model_path, pin.utils.fromListToVectorOfString(hint_list), pin.GeometryType.COLLISION)

        col = collision_model.geometryObjects[1]
        self.assertTrue(col.meshPath == expected_mesh_path)

if __name__ == '__main__':
    unittest.main()
