import pinocchio as se3
from pinocchio.utils import np, zero

from test_case import TestCase


class TestModel(TestCase):
    def setUp(self):
        self.model = se3.buildSampleModelHumanoidRandom()

    def test_empty_model_sizes(self):
        model = se3.Model()
        self.assertEqual(model.nbodies, 1)
        self.assertEqual(model.nq, 0)
        self.assertEqual(model.nv, 0)

    def test_model(self):
        model = self.model
        nb = 28  # We should have 28 bodies, thus 27 joints, one of them a free-flyer.
        self.assertEqual(model.nbodies, nb)
        self.assertEqual(model.nq, nb - 1 + 6)
        self.assertEqual(model.nv, nb - 1 + 5)

    def test_inertias(self):
        model = self.model
        model.inertias[1] = model.inertias[2]
        self.assertApprox(model.inertias[1].np, model.inertias[2].np)

    def test_placements(self):
        model = self.model
        model.jointPlacements[1] = model.jointPlacements[2]
        self.assertApprox(model.jointPlacements[1].np, model.jointPlacements[2].np)
        self.assertEqual(model.parents[0], 0)
        self.assertEqual(model.parents[1], 0)
        model.parents[2] = model.parents[1]
        self.assertEqual(model.parents[2], model.parents[1])
        self.assertEqual(model.names[0], "universe")

    def test_gravity(self):
        self.assertApprox(self.model.gravity.np, np.matrix('0 0 -9.81 0 0 0').T)

    def test_rnea(self):
        model = self.model
        data = model.createData()

        q = zero(model.nq)
        qdot = zero(model.nv)
        qddot = zero(model.nv)
        for i in range(model.nbodies):
            data.a[i] = se3.Motion.Zero()

        se3.rnea(model, data, q, qdot, qddot)
        for i in range(model.nbodies):
            self.assertApprox(data.v[i].np, zero(6))
        self.assertApprox(data.a_gf[0].np, -model.gravity.np)
        self.assertApprox(data.f[-1], model.inertias[-1] * data.a_gf[-1])

if __name__ == '__main__':
    unittest.main()
