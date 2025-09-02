import os
import unittest
from pathlib import Path

import pinocchio as pin


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):
    def setUp(self):
        self.model_dir = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
        self.model_path = self.model_dir / "romeo_description/urdf/romeo.urdf"

    def test_load(self):
        pin.buildModelFromUrdf(self.model_path)
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_self_load(self):
        model = pin.Model()
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer(), model)
        pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

    def test_xml(self):
        with self.model_path.open() as model:
            file_content = model.read()

        model_ref = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        model = pin.buildModelFromXML(file_content, pin.JointModelFreeFlyer())

        self.assertEqual(model, model_ref)

        model_self = pin.Model()
        pin.buildModelFromXML(file_content, pin.JointModelFreeFlyer(), model_self)
        self.assertEqual(model_self, model_ref)

    def test_pickle(self):
        import pickle

        model_path = self.model_dir / "ur_description/urdf/ur5_robot.urdf"

        model = pin.buildModelFromUrdf(model_path)
        filename = Path("model.pickle")
        with filename.open("wb") as f:
            pickle.dump(model, f)

        with filename.open("rb") as f:
            model_copy = pickle.load(f)

        self.assertTrue(model == model_copy)


if __name__ == "__main__":
    unittest.main()
