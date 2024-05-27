import os
import unittest

import pinocchio as pin


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestBuildGeomFromUrdfMemoryCheck(unittest.TestCase):
    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))
        self.model_dir = os.path.abspath(
            os.path.join(self.current_file, "../../models/")
        )
        self.model_path = os.path.abspath(
            os.path.join(
                self.model_dir,
                "example-robot-data/robots/ur_description/urdf/ur5_robot.urdf",
            )
        )

    def test_load(self):
        model = pin.buildModelFromUrdf(self.model_path)
        for _ in range(2):
            pin.buildGeomFromUrdf(
                model,
                self.model_path,
                pin.COLLISION,
                package_dirs=self.model_dir,
            )


if __name__ == "__main__":
    unittest.main()
