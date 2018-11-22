import unittest
import pinocchio as pin
import numpy as np

class TestJointCompositeBindings(unittest.TestCase):

    def test_basic(self):
        jc = pin.JointModelComposite()
        self.assertTrue(hasattr(jc,'joints'))
        self.assertTrue(hasattr(jc,'njoints'))
        self.assertTrue(hasattr(jc,'jointPlacements'))

    def test_empty_constructor(self):
        jc = pin.JointModelComposite()
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)
        self.assertTrue(jc.njoints==len(jc.joints))

    def test_reserve_constructor(self):
        jc = pin.JointModelComposite(2)
        self.assertTrue(jc.nq==0)
        self.assertTrue(len(jc.joints)==0)
        self.assertTrue(jc.njoints==len(jc.joints))

    def test_joint_constructor(self):
        j1 = pin.JointModelRX()
        self.assertTrue(j1.nq==1)

        jc1 = pin.JointModelComposite(j1)
        self.assertTrue(jc1.nq==1)
        self.assertTrue(len(jc1.joints)==1)
        self.assertTrue(jc1.njoints==len(jc1.joints))

        j2 = pin.JointModelRX()
        self.assertTrue(j2.nq==1)

        jc2 = pin.JointModelComposite(j1,pin.SE3.Random())
        self.assertTrue(jc2.nq==1)
        self.assertTrue(len(jc2.joints)==1)
        self.assertTrue(jc2.njoints==len(jc2.joints))

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
        self.assertTrue(jc.njoints==len(jc.joints))

        jc.addJoint(j1)
        self.assertTrue(jc.nq==1)
        self.assertTrue(len(jc.joints)==1)
        self.assertTrue(jc.njoints==len(jc.joints))

        jc.addJoint(j2)
        self.assertTrue(jc.nq==2)
        self.assertTrue(len(jc.joints)==2)
        self.assertTrue(jc.njoints==len(jc.joints))

        jc.addJoint(j3,pin.SE3.Random())
        self.assertTrue(jc.nq==3)
        self.assertTrue(jc.njoints==len(jc.joints))
        
    def test_add_joint_return(self):
        jc1 = pin.JointModelComposite()
        jc2 = jc1.addJoint(pin.JointModelRX())
        jc3 = jc2.addJoint(pin.JointModelRY())
        jc4 = jc1.addJoint(pin.JointModelRZ())
        self.assertTrue(jc1.njoints==3)
        self.assertTrue(jc2.njoints==3)
        self.assertTrue(jc3.njoints==3)
        self.assertTrue(jc4.njoints==3)
        
        del jc1
        del jc3
        del jc4
        self.assertTrue(jc2.njoints==3)

    def test_add_joint_concat(self):
        j1 = pin.JointModelRX()
        self.assertTrue(j1.nq==1)
        j2 = pin.JointModelRY()
        self.assertTrue(j2.nq==1)
        j3 = pin.JointModelRZ()
        self.assertTrue(j3.nq==1)

        jc = pin.JointModelComposite(j1).addJoint(j2,pin.SE3.Random()).addJoint(j3)

        self.assertTrue(jc.nq==3)
        self.assertTrue(len(jc.joints)==3)
        self.assertTrue(jc.njoints==len(jc.joints))

if __name__ == '__main__':
    unittest.main()
