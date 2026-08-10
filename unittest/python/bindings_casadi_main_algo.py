import sys
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
import casadi
import numpy as np
import pinocchio as pin
from casadi import SX
from pinocchio import casadi as cpin
from test_case import PinocchioTestCase as TestCase


class TestMainAlgos(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        qmax = np.full((self.model.nq, 1), np.pi)
        self.q = pin.randomConfiguration(self.model, -qmax, qmax)
        self.v = np.random.rand(self.model.nv)
        self.a = np.random.rand(self.model.nv)
        self.tau = np.random.rand(self.model.nv)

        self.cq = SX.sym("q", self.cmodel.nq, 1)
        self.cv = SX.sym("v", self.cmodel.nv, 1)
        self.ca = SX.sym("a", self.cmodel.nv, 1)
        self.ctau = SX.sym("tau", self.cmodel.nv, 1)

        self.fext = []
        self.cfext = []
        for i in range(self.model.njoints):
            self.fext.append(pin.Force.Random())
            fname = "f" + str(i)
            self.cfext.append(cpin.Force(SX.sym(fname, 6, 1)))

        self.fext[0].setZero()

    def test_crba(self):
        model = self.model
        data = self.data

        cmodel = self.cmodel
        cdata = self.cdata

        cM = cpin.crba(cmodel, cdata, self.cq)
        f_crba = casadi.Function("crba", [self.cq], [cM])

        arg_crba = [list(self.q)]
        M = f_crba.call(arg_crba)[0].full()

        M_ref = pin.crba(model, data, self.q)

        self.assertApprox(M, M_ref)

    def test_rnea(self):
        model = self.model
        data = self.data

        cmodel = self.cmodel
        cdata = self.cdata

        ctau = cpin.rnea(cmodel, cdata, self.cq, self.cv, self.ca)
        f_rnea = casadi.Function("rnea", [self.cq, self.cv, self.ca], [ctau])

        arg_rnea = [list(self.q), list(self.v), list(self.a)]
        tau = f_rnea.call(arg_rnea)[0].full()

        tau_ref = pin.rnea(model, data, self.q, self.v, self.a)

        self.assertApprox(tau, tau_ref)

        ctau_fext = cpin.rnea(cmodel, cdata, self.cq, self.cv, self.ca, self.cfext)
        carg_in = [self.cq, self.cv, self.ca]
        for f in self.cfext:
            carg_in += [f.vector]
        f_rnea_fext = casadi.Function("rnea_fext", carg_in, [ctau_fext])

        arg_rnea_fext = arg_rnea
        for f in self.fext:
            arg_rnea_fext += [f.vector]
        tau_fext = f_rnea_fext.call(arg_rnea_fext)[0].full()

        tau_fext_ref = pin.rnea(model, data, self.q, self.v, self.a, self.fext)

        self.assertApprox(tau_fext, tau_fext_ref)

    def test_aba(self):
        model = self.model
        data = self.data

        cmodel = self.cmodel
        cdata = self.cdata

        cddq = cpin.aba(cmodel, cdata, self.cq, self.cv, self.ctau)
        f_aba = casadi.Function("aba", [self.cq, self.cv, self.ctau], [cddq])

        arg_aba = [list(self.q), list(self.v), list(self.tau)]
        a = f_aba.call(arg_aba)[0].full()

        a_ref = pin.aba(model, data, self.q, self.v, self.tau)

        self.assertApprox(a, a_ref)

        cddq_fext = cpin.aba(cmodel, cdata, self.cq, self.cv, self.ctau, self.cfext)
        carg_in = [self.cq, self.cv, self.ctau]
        for f in self.cfext:
            carg_in += [f.vector]
        f_aba_fext = casadi.Function("aba_fext", carg_in, [cddq_fext])

        arg_aba_fext = arg_aba
        for f in self.fext:
            arg_aba_fext += [f.vector]
        a_fext = f_aba_fext.call(arg_aba_fext)[0].full()

        a_fext_ref = pin.aba(model, data, self.q, self.v, self.tau, self.fext)

        self.assertApprox(a_fext, a_fext_ref)

    def test_computeMinverse(self):
        model = self.model
        Minv = pin.computeMinverse(model, self.data, self.q)

        data2 = model.createData()
        M = pin.crba(model, data2, self.q)

        self.assertApprox(np.linalg.inv(M), Minv)


if __name__ == "__main__":
    unittest.main()
