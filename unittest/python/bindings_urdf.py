import unittest
import pinocchio as pin
import os

@unittest.skipUnless(pin.WITH_URDFDOM,"Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):

    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))
        self.model_dir = os.path.abspath(os.path.join(self.current_file, "../../models/others/robots"))
        self.model_path = os.path.abspath(os.path.join(self.model_dir, "romeo_description/urdf/romeo.urdf"))

    def test_load(self):
        model = pin.buildModelFromUrdf(self.model_path)
        model_ff = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_self_load(self):
        model = pin.Model()
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer(), model)
        model_ref = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_xml(self):
        file_content = open(self.model_path,"r").read()
        
        model_ref = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        model = pin.buildModelFromXML(file_content,pin.JointModelFreeFlyer())

        self.assertEqual(model,model_ref)

        model_self = pin.Model()
        pin.buildModelFromXML(file_content,pin.JointModelFreeFlyer(),model_self)
        self.assertEqual(model_self,model_ref)

if __name__ == '__main__':
    unittest.main()
