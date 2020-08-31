import unittest
import pinocchio as pin
from pinocchio.utils import *
from numpy.linalg import norm

def df_dq(model,func,q,h=1e-9):
    """ Perform df/dq by num_diff. q is in the lie manifold.
    :params func: function to differentiate f : np.matrix -> np.matrix
    :params q: configuration value at which f is differentiated. type np.matrix
    :params h: eps
    
    :returns df/dq
    """
    dq = zero(model.nv)
    f0 = func(q)
    res = zero([len(f0),model.nv])
    for iq in range(model.nv):
        dq[iq] = h
        res[:,iq] = (func(pin.integrate(model,q,dq)) - f0)/h
        dq[iq] = 0
    return res

class TestVComDerivativesBindings(unittest.TestCase):
    def setUp(self):
        self.rmodel = rmodel = pin.buildSampleModelHumanoid()
        self.rdata  = rmodel.createData()

        self.q = pin.randomConfiguration(rmodel)
        self.vq = rand(rmodel.nv)*2-1
        self.aq = zero(rmodel.nv)
        self.dq = rand(rmodel.nv)*2-1

        self.precision = 1e-8
        
    def test_numdiff(self):
        rmodel,rdata = self.rmodel,self.rdata
        q,vq,aq,dq= self.q,self.vq,self.aq,self.dq

        #### Compute d/dq VCOM with the algo.
        pin.computeAllTerms(rmodel,rdata,q,vq)
        pin.computeForwardKinematicsDerivatives(rmodel,rdata,q,vq,aq)
        dvc_dq = pin.getCenterOfMassVelocityDerivatives(rmodel,rdata)
        
        #### Approximate d/dq VCOM by finite diff.
        def calc_vc(q,vq):
            """ Compute COM velocity """
            pin.centerOfMass(rmodel,rdata,q,vq)
            return rdata.vcom[0].copy()
        dvc_dqn = df_dq(rmodel,lambda _q: calc_vc(_q,vq),q)
        
        self.assertTrue(np.allclose(dvc_dq,dvc_dqn,atol=np.sqrt(self.precision)))

if __name__ == '__main__':
    unittest.main()

