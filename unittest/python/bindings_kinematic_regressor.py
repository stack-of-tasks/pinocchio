import unittest
from test_case import PinocchioTestCase as TestCase
import pinocchio as pin


class TestKinematicRegressorBindings(TestCase):
    def test_all(self):
        model = pin.buildSampleModelHumanoidRandom()

        joint_name = "larm6_joint"
        joint_id = model.getJointId(joint_name)
        frame_id = model.addBodyFrame("test_body", joint_id, pin.SE3.Identity(), -1)

        data = model.createData()

        model.lowerPositionLimit[:7] = -1.0
        model.upperPositionLimit[:7] = 1.0

        q = pin.randomConfiguration(model)
        pin.forwardKinematics(model, data, q)

        R1 = pin.computeJointKinematicRegressor(
            model, data, joint_id, pin.ReferenceFrame.LOCAL, pin.SE3.Identity()
        )
        R2 = pin.computeJointKinematicRegressor(
            model, data, joint_id, pin.ReferenceFrame.LOCAL
        )

        self.assertApprox(R1, R2)

        R3 = pin.computeFrameKinematicRegressor(
            model, data, frame_id, pin.ReferenceFrame.LOCAL
        )
        self.assertApprox(R1, R3)


if __name__ == "__main__":
    unittest.main()
