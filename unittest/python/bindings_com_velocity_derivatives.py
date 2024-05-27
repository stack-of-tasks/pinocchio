import unittest
import pinocchio as pin
from pinocchio.utils import *
from numpy.linalg import norm


def df_dq(model, func, q, h=1e-9):
    """Perform df/dq by num_diff. q is in the lie manifold.
    :params func: function to differentiate f : np.array -> np.array
    :params q: configuration value at which f is differentiated. type np.array
    :params h: eps

    :returns df/dq
    """
    dq = zero(model.nv)
    f0 = func(q)
    res = zero([len(f0), model.nv])
    for iq in range(model.nv):
        dq[iq] = h
        res[:, iq] = (func(pin.integrate(model, q, dq)) - f0) / h
        dq[iq] = 0
    return res


class TestVComDerivativesBindings(unittest.TestCase):
    def setUp(self):
        self.rmodel = rmodel = pin.buildSampleModelHumanoid()
        self.rdata = rmodel.createData()
        self.rdata_fd = rmodel.createData()

        self.rmodel.lowerPositionLimit[:3] = -1.0
        self.rmodel.upperPositionLimit[:3] = -1.0
        self.q = pin.randomConfiguration(rmodel)
        self.vq = rand(rmodel.nv) * 2 - 1

        self.precision = 1e-8

    def test_numdiff(self):
        rmodel, rdata = self.rmodel, self.rdata
        rdata_fd = self.rdata_fd
        q, vq = self.q, self.vq

        #### Compute d/dq VCOM with the algo.
        pin.computeAllTerms(rmodel, rdata, q, vq)
        dvc_dq = pin.getCenterOfMassVelocityDerivatives(rmodel, rdata)

        #### Approximate d/dq VCOM by finite diff.
        def calc_vc(q, vq):
            """Compute COM velocity"""
            pin.centerOfMass(rmodel, rdata_fd, q, vq)
            return rdata_fd.vcom[0].copy()

        dvc_dqn = df_dq(rmodel, lambda _q: calc_vc(_q, vq), q)

        self.assertTrue(np.allclose(dvc_dq, dvc_dqn, atol=np.sqrt(self.precision)))


if __name__ == "__main__":
    unittest.main()
