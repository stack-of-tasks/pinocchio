import unittest
import pinocchio as pin
import numpy as np

from test_case import PinocchioTestCase

class TestFrameBindings(PinocchioTestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.joint_idx = self.model.getJointId("rarm2_joint") if self.model.existJointName("rarm2_joint") else (self.model.njoints-1)

        self.data = self.model.createData()
        self.q = pin.randomConfiguration(self.model)
        self.v = np.random.rand((self.model.nv))
        self.a = np.random.rand((self.model.nv))

    def tearDown(self):
        del self.model

    def test_derivatives(self):
        model = self.model
        data = self.data

        q = self.q
        v = self.v
        a = self.a

        pin.computeForwardKinematicsDerivatives(model,data,q,v,a)

        pin.getJointVelocityDerivatives(model,data,self.joint_idx,pin.WORLD)
        pin.getJointVelocityDerivatives(model,data,self.joint_idx,pin.LOCAL)
        pin.getJointVelocityDerivatives(model,data,self.joint_idx,pin.LOCAL_WORLD_ALIGNED)

        pin.getJointAccelerationDerivatives(model,data,self.joint_idx,pin.WORLD)
        pin.getJointAccelerationDerivatives(model,data,self.joint_idx,pin.LOCAL)
        pin.getJointAccelerationDerivatives(model,data,self.joint_idx,pin.LOCAL_WORLD_ALIGNED)

if __name__ == '__main__':
    unittest.main()
