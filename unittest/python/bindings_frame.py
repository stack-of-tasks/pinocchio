import unittest
import pinocchio as se3
import numpy as np

class TestFrameBindings(unittest.TestCase):

    def setUp(self):
        self.model = se3.buildSampleModelHumanoidRandom()
        self.parent_idx = self.model.getJointId("rarm2_joint") if self.model.existJointName("rarm2_joint") else (self.model.njoints-1)
        self.frame_name = self.model.names[self.parent_idx] + "_frame"
        self.frame_placement = se3.SE3.Random()
        self.frame_type = se3.FrameType.OP_FRAME
        self.model.addFrame(se3.Frame(self.frame_name, self.parent_idx, 0, self.frame_placement, self.frame_type))
        self.frame_idx = self.model.getFrameId(self.frame_name)

    def tearDown(self):
        del self.model

    def test_type_get_set(self):
        f = self.model.frames[self.frame_idx]
        self.assertTrue(f.type == self.frame_type)
        f.type = se3.FrameType.BODY
        self.assertTrue(f.type == se3.FrameType.BODY)

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
        new_placement = se3.SE3.Random()
        f.placement = new_placement
        self.assertTrue(np.allclose(f.placement.homogeneous, new_placement.homogeneous))

if __name__ == '__main__':
    unittest.main()
