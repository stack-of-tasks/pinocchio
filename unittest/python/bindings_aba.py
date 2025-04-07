import unittest

import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestABA(TestCase):
    def setUp(self):
        # Set up non mimic model
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq, 1), np.pi)
        self.q = pin.randomConfiguration(self.model, -qmax, qmax)
        self.v = np.random.rand(self.model.nv)
        self.ddq = np.random.rand(self.model.nv)

        self.fext = []
        for _ in range(self.model.njoints):
            self.fext.append(pin.Force.Random())

        # Set up mimic model
        self.model_mimic = pin.buildSampleModelManipulator(True)
        self.data_mimic = self.model_mimic.createData()
        self.q_mimic = np.zeros((self.model_mimic.nq,))

    def test_aba(self):
        model = self.model
        ddq = pin.aba(self.model, self.data, self.q, self.v, self.ddq)

        null_fext = pin.StdVec_Force()
        for _ in range(model.njoints):
            null_fext.append(pin.Force.Zero())

        ddq_null_fext = pin.aba(
            self.model, self.data, self.q, self.v, self.ddq, null_fext
        )
        self.assertApprox(ddq_null_fext, ddq)

        null_fext_list = []
        for f in null_fext:
            null_fext_list.append(f)

        print("size:", len(null_fext_list))
        ddq_null_fext_list = pin.aba(
            self.model, self.data, self.q, self.v, self.ddq, null_fext_list
        )
        self.assertApprox(ddq_null_fext_list, ddq)

    def test_computeMinverse(self):
        model = self.model
        Minv = pin.computeMinverse(model, self.data, self.q)

        data2 = model.createData()
        M = pin.crba(model, data2, self.q)

        self.assertApprox(np.linalg.inv(M), Minv)

    def test_assert_mimic_not_supported_function(self):
        self.assertRaises(
            RuntimeError,
            pin.computeMinverse,
            self.model_mimic,
            self.data_mimic,
            self.q_mimic,
        )


if __name__ == "__main__":
    unittest.main()
