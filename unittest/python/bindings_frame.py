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

    def tearDown(self):
        del self.model

    def test_type_get_set(self):
        f = self.model.frames[self.frame_idx]
        self.assertTrue(f.type == self.frame_type)
        f.type = pin.FrameType.BODY
        self.assertTrue(f.type == pin.FrameType.BODY)

    def test_name_get_set(self):
        f = self.model.frames[self.frame_idx]
        self.assertTrue(f.name == self.frame_name)
        f.name = 'new_hip_frame'
        self.assertTrue(f.name == 'new_hip_frame')

    def test_parent_get_set(self):
        f = self.model.frames[self.frame_idx]
        self.assertTrue(f.parent == self.parent_idx)
        newparent = self.parent_idx-1
        f.parent = newparent
        self.assertTrue(f.parent == newparent)

    def test_placement_get_set(self):
        f = self.model.frames[self.frame_idx]
        self.assertTrue(np.allclose(f.placement.homogeneous, self.frame_placement.homogeneous))
        new_placement = pin.SE3.Random()
        f.placement = new_placement
        self.assertTrue(np.allclose(f.placement.homogeneous, new_placement.homogeneous))

    def test_getters(self):
        data = self.model.createData()
        q = pin.randomConfiguration(self.model)
        v = np.random.rand(self.model.nv)
        a = np.random.rand(self.model.nv)
        pin.forwardKinematics(self.model, data, q, v, a)

        T = pin.updateFramePlacement(self.model, data, self.frame_idx)
        self.assertApprox(T, data.oMi[self.parent_idx].act(self.frame_placement))

        v = pin.getFrameVelocity(self.model, data, self.frame_idx)
        self.assertApprox(v, self.frame_placement.actInv(data.v[self.parent_idx]))
        v = pin.getFrameVelocity(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL)
        self.assertApprox(v, self.frame_placement.actInv(data.v[self.parent_idx]))
        v = pin.getFrameVelocity(self.model, data, self.frame_idx, pin.ReferenceFrame.WORLD)
        self.assertApprox(v, data.oMi[self.parent_idx].act(data.v[self.parent_idx]))
        v = pin.getFrameVelocity(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.assertApprox(v, pin.SE3(T.rotation, np.zeros(3)).act(self.frame_placement.actInv(data.v[self.parent_idx])))

        a = pin.getFrameAcceleration(self.model, data, self.frame_idx)
        self.assertApprox(a, self.frame_placement.actInv(data.a[self.parent_idx]))
        a = pin.getFrameAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL)
        self.assertApprox(a, self.frame_placement.actInv(data.a[self.parent_idx]))
        a = pin.getFrameAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.WORLD)
        self.assertApprox(a, data.oMi[self.parent_idx].act(data.a[self.parent_idx]))
        a = pin.getFrameAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.assertApprox(a, pin.SE3(T.rotation, np.zeros(3)).act(self.frame_placement.actInv(data.a[self.parent_idx])))

        a = pin.getFrameClassicalAcceleration(self.model, data, self.frame_idx)
        a = pin.getFrameClassicalAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL)
        a = pin.getFrameClassicalAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.WORLD)
        a = pin.getFrameClassicalAcceleration(self.model, data, self.frame_idx, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    def test_frame_algo(self):
        model = self.model
        data = model.createData()

        q = pin.neutral(model)
        v = np.random.rand((model.nv))
        frame_id = self.frame_idx

        J1 = pin.computeFrameJacobian(model,data,q,frame_id)
        J2 = pin.computeFrameJacobian(model,data,q,frame_id,pin.LOCAL)

        self.assertApprox(J1,J2)
        data2 = model.createData()

        pin.computeJointJacobians(model,data2,q)
        J3 = pin.getFrameJacobian(model,data2,frame_id,pin.LOCAL)
        self.assertApprox(J1,J3)

        dJ1 = pin.frameJacobianTimeVariation(model,data,q,v,frame_id,pin.LOCAL)

        data3 = model.createData()
        pin.computeJointJacobiansTimeVariation(model,data3,q,v)

        dJ2 = pin.getFrameJacobianTimeVariation(model,data3,frame_id,pin.LOCAL)
        self.assertApprox(dJ1,dJ2)

if __name__ == '__main__':
    unittest.main()
