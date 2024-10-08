import unittest
from pathlib import Path

import pinocchio as pin


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestBuildGeomFromUrdfMemoryCheck(unittest.TestCase):
    def setUp(self):
        self.current_dir = Path(__file__).parent
        self.model_dir = self.current_dir / "../../models"
        self.model_path = (
            self.model_dir
            / "example-robot-data/robots/ur_description/urdf/ur5_robot.urdf"
        )

    def test_load(self):
        model = pin.buildModelFromUrdf(self.model_path, False)
        for _ in range(2):
            pin.buildGeomFromUrdf(
                model,
                self.model_path,
                pin.COLLISION,
                package_dirs=self.model_dir,
            )


if __name__ == "__main__":
    unittest.main()
