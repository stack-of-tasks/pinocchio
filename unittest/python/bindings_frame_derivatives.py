import unittest
import pinocchio as pin
import numpy as np

from test_case import PinocchioTestCase

class TestFrameBindings(PinocchioTestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.parent_idx = self.model.getJointId("rarm2_joint") if self.model.existJointName("rarm2_joint") else (self.model.njoints-1)
        self.frame_name = self.model.names[self.parent_idx] + "_frame"
        self.frame_placement = pin.SE3.Random()
        self.frame_type = pin.FrameType.OP_FRAME
        self.model.addFrame(pin.Frame(self.frame_name, self.parent_idx, 0, self.frame_placement, self.frame_type))
        self.frame_idx = self.model.getFrameId(self.frame_name)

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

        pin.getFrameVelocityDerivatives(model,data,self.frame_idx,pin.WORLD)
        pin.getFrameVelocityDerivatives(model,data,self.frame_idx,pin.LOCAL)
        pin.getFrameVelocityDerivatives(model,data,self.frame_idx,pin.LOCAL_WORLD_ALIGNED)

        pin.getFrameAccelerationDerivatives(model,data,self.frame_idx,pin.WORLD)
        pin.getFrameAccelerationDerivatives(model,data,self.frame_idx,pin.LOCAL)
        pin.getFrameAccelerationDerivatives(model,data,self.frame_idx,pin.LOCAL_WORLD_ALIGNED)

if __name__ == '__main__':
    unittest.main()
