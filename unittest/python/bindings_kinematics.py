import unittest
import pinocchio as pin
import numpy as np

from test_case import PinocchioTestCase

class TestKinematicsBindings(PinocchioTestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.joint_idx = self.model.getJointId("rarm2_joint") if self.model.existJointName("rarm2_joint") else (self.model.njoints-1)

    def test_getters(self):
        data = self.model.createData()
        q = pin.randomConfiguration(self.model)
        v = np.random.rand(self.model.nv)
        a = np.random.rand(self.model.nv)
        pin.forwardKinematics(self.model, data, q, v, a)

        T = data.oMi[self.joint_idx]

        v = pin.getVelocity(self.model, data, self.joint_idx)
        self.assertApprox(v, data.v[self.joint_idx])
        v = pin.getVelocity(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL)
        self.assertApprox(v, data.v[self.joint_idx])
        v = pin.getVelocity(self.model, data, self.joint_idx, pin.ReferenceFrame.WORLD)
        self.assertApprox(v, data.oMi[self.joint_idx].act(data.v[self.joint_idx]))
        v = pin.getVelocity(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.assertApprox(v, pin.SE3(T.rotation, np.zeros(3)).act(data.v[self.joint_idx]))

        a = pin.getAcceleration(self.model, data, self.joint_idx)
        self.assertApprox(a, data.a[self.joint_idx])
        a = pin.getAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL)
        self.assertApprox(a, data.a[self.joint_idx])
        a = pin.getAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.WORLD)
        self.assertApprox(a, data.oMi[self.joint_idx].act(data.a[self.joint_idx]))
        a = pin.getAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.assertApprox(a, pin.SE3(T.rotation, np.zeros(3)).act(data.a[self.joint_idx]))

        a = pin.getClassicalAcceleration(self.model, data, self.joint_idx)
        a = pin.getClassicalAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL)
        a = pin.getClassicalAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.WORLD)
        a = pin.getClassicalAcceleration(self.model, data, self.joint_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

if __name__ == '__main__':
    unittest.main()
