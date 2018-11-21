import unittest
import pinocchio as pin
import numpy as np

class TestJointCompositeBindings(unittest.TestCase):

    def test_basic(self):
        jc = pin.JointModelComposite()
        self.assertTrue(hasattr(jc,'joints'))

    def test_empty_constructor(self):
        jc = pin.JointModelComposite()
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

    def test_reserve_constructor(self):
        jc = pin.JointModelComposite(2)
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

    def test_joint_constructor(self):
        j1 = pin.JointModelRX()
        self.assertTrue(j1.nq==1)

        jc1 = pin.JointModelComposite(j1)
        self.assertTrue(jc1.nq==1)
        self.assertTrue(len(jc1.joints)==1)

        j2 = pin.JointModelRX()
        self.assertTrue(j2.nq==1)

        jc2 = pin.JointModelComposite(j1,pin.SE3.Identity())
        self.assertTrue(jc2.nq==1)
        self.assertTrue(len(jc2.joints)==1)

    def test_add_joint(self):
        j1 = pin.JointModelRX()
        self.assertTrue(j1.nq==1)
        j2 = pin.JointModelRY()
        self.assertTrue(j2.nq==1)
        j3 = pin.JointModelRZ()
        self.assertTrue(j3.nq==1)

        jc = pin.JointModelComposite(2)
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)

        jc.addJoint(j1)
        self.assertTrue(jc.nq==1)
        self.assertTrue(len(jc.joints)==1)

        jc.addJoint(j2)
        self.assertTrue(jc.nq==2)
        self.assertTrue(len(jc.joints)==2)

        jc.addJoint(j3,pin.SE3.Identity())
        self.assertTrue(jc.nq==3)
        self.assertTrue(len(jc.joints)==3)
        

if __name__ == '__main__':
    unittest.main()
