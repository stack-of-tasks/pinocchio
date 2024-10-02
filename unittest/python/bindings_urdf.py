import os
import unittest

import pinocchio as pin


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):
    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))
        self.model_dir = os.path.abspath(
            os.path.join(self.current_file, "../../models/example-robot-data/robots")
        )
        self.model_path = os.path.abspath(
            os.path.join(self.model_dir, "romeo_description/urdf/romeo.urdf")
        )

    def test_load(self):
        pin.buildModelFromUrdf(self.model_path)
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_self_load(self):
        model = pin.Model()
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer(), model)
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_xml(self):
        with open(self.model_path) as model:
            file_content = model.read()

        model_ref = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        model = pin.buildModelFromXML(file_content, pin.JointModelFreeFlyer())

        self.assertEqual(model, model_ref)

        model_self = pin.Model()
        pin.buildModelFromXML(file_content, pin.JointModelFreeFlyer(), model_self)
        self.assertEqual(model_self, model_ref)

    def test_pickle(self):
        import pickle

        model_dir = os.path.abspath(
            os.path.join(self.current_file, "../../models/example-robot-data/robots")
        )
        model_path = os.path.abspath(
            os.path.join(model_dir, "ur_description/urdf/ur5_robot.urdf")
        )

        model = pin.buildModelFromUrdf(model_path)
        filename = "model.pickle"
        with open(filename, "wb") as f:
            pickle.dump(model, f)

        with open(filename, "rb") as f:
            model_copy = pickle.load(f)

        self.assertTrue(model == model_copy)


if __name__ == "__main__":
    unittest.main()
