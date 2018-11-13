import unittest
import pinocchio as se3
import numpy as np

class TestJointCompositeBindings(unittest.TestCase):

    def test_basic(self):
        jc = se3.JointModelComposite()
        self.assertTrue(hasattr(jc,'joints'))

    def test_empty_constructor(self):
        jc = se3.JointModelComposite()
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

    def test_reserve_constructor(self):
        jc = se3.JointModelComposite(2)
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

    def test_add_joint(self):
        j1 = se3.JointModelRX()
        self.assertTrue(j1.nq==1)
        j2 = se3.JointModelRY()
        self.assertTrue(j2.nq==1)
        j3 = se3.JointModelRZ()
        self.assertTrue(j3.nq==1)

        jc = se3.JointModelComposite(2)
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

        jc.addJoint(j1)
        self.assertTrue(jc.nq==1)
        self.assertTrue(len(jc.joints)==1)

        jc.addJoint(j2)
        self.assertTrue(jc.nq==2)
        self.assertTrue(len(jc.joints)==2)

        jc.addJoint(j3,se3.SE3.Identity())
        self.assertTrue(jc.nq==3)
        self.assertTrue(len(jc.joints)==3)
        

if __name__ == '__main__':
    unittest.main()
