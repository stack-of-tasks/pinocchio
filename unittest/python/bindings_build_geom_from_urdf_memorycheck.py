import os
import unittest
from pathlib import Path

import pinocchio as pin


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestBuildGeomFromUrdfMemoryCheck(unittest.TestCase):
    def setUp(self):
        self.model_dir = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
        self.model_path = self.model_dir / "ur_description/urdf/ur5_robot.urdf"

    def test_load(self):
        model = pin.buildModelFromUrdf(self.model_path)
        for _ in range(2):
            pin.buildGeomFromUrdf(
                model,
                self.model_path,
                pin.COLLISION,
                package_dirs=self.model_dir.parent.parent,
            )


if __name__ == "__main__":
    unittest.main()
