import copy
import importlib.util
import unittest

import numpy as np
import pinocchio as pin

coal_found = importlib.util.find_spec("coal") is not None
if coal_found:
    import coal

COPY_ROUTES = {
    "copy()": lambda obj: obj.copy(),
    "copy.copy": copy.copy,
    "copy.deepcopy": copy.deepcopy,
}


def sameValue(a, b):
    return np.array_equal(np.asarray(a), np.asarray(b))


class TestCopy(unittest.TestCase):
    def setUp(self):
        self.model = pin.Model()
        self.joint_id = self.model.addJoint(
            0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "body"
        )
        self.model.appendBodyToJoint(
            self.joint_id, pin.Inertia.Identity(), pin.SE3.Identity()
        )

    def assertCopies(self, obj, edit, read):
        """Each copy route returns an independent object of the same class, equal to
        `obj`: `edit` changes what `read` returns on the copy, and not on `obj`."""
        original_value = read(obj)
        for route, copy_fn in COPY_ROUTES.items():
            with self.subTest(cls=type(obj).__name__, route=route):
                other = copy_fn(obj)
                self.assertIs(type(other), type(obj))
                self.assertIsNot(other, obj)
                if "__eq__" in vars(type(obj)):
                    self.assertTrue(other == obj)
                edit(other)
                self.assertFalse(sameValue(read(other), original_value))
                self.assertTrue(sameValue(read(obj), original_value))

    def test_anchor_constraint(self):
        anchor = pin.FrameAnchorConstraintModel(
            self.model, self.joint_id, pin.SE3.Random(), 0, pin.SE3.Random()
        )
        self.assertCopies(
            anchor,
            lambda c: c.setCompliance(np.full(6, 1e-6)),
            lambda c: c.retrieveCompliance(),
        )
        self.assertCopies(
            anchor.createData(),
            lambda c: setattr(c, "oMc1", pin.SE3.Random()),
            lambda c: c.oMc1.homogeneous,
        )

    def test_contact_constraint(self):
        contact = pin.PointContactConstraintModel(
            self.model, 0, pin.SE3.Identity(), self.joint_id, pin.SE3.Identity()
        )
        self.assertCopies(
            contact, lambda c: setattr(c, "name", "other"), lambda c: c.name
        )
        self.assertCopies(
            contact.createData(),
            lambda c: setattr(c, "oMc2", pin.SE3.Random()),
            lambda c: c.oMc2.homogeneous,
        )

        generic = pin.ConstraintModel(contact)
        self.assertCopies(
            generic,
            lambda c: c.setBaumgarteCorrectorParameters(
                pin.BaumgarteCorrectorParameters(1.0, 2.0)
            ),
            lambda c: (
                c.baumgarte_corrector_parameters.Kp,
                c.baumgarte_corrector_parameters.Kd,
            ),
        )

        models = pin.StdVec_ConstraintModel()
        models.append(generic)
        self.assertCopies(models, lambda c: c.append(generic), len)

    def test_baumgarte_corrector_parameters(self):
        self.assertCopies(
            pin.BaumgarteCorrectorParameters(1.0, 2.0),
            lambda c: setattr(c, "Kp", 3.0),
            lambda c: c.Kp,
        )

    def test_proximal_settings(self):
        self.assertCopies(
            pin.ProximalSettings(1e-6, 1e-3, 10),
            lambda c: setattr(c, "mu", 1.0),
            lambda c: c.mu,
        )

    def test_solver_settings_results_and_stats(self):
        for settings in (pin.PGSSolverSettings(), pin.ADMMSolverSettings()):
            self.assertCopies(
                settings,
                lambda c: setattr(c, "max_iterations", 1234),
                lambda c: c.max_iterations,
            )
        for result in (pin.PGSSolverResult(), pin.ADMMSolverResult()):
            self.assertCopies(
                result,
                lambda c: setattr(c, "iterations", 42),
                lambda c: c.iterations,
            )
        for stats in (pin.PGSSolverStats(), pin.ADMMSolverStats()):
            self.assertCopies(
                stats,
                lambda c: setattr(c, "iterations", 42),
                lambda c: c.iterations,
            )

    def test_joints(self):
        joint_model = pin.JointModelRevoluteUnaligned(np.array([1.0, 0.0, 0.0]))
        self.assertCopies(
            joint_model,
            lambda c: setattr(c, "axis", np.array([0.0, 1.0, 0.0])),
            lambda c: c.axis,
        )
        self.assertCopies(
            pin.JointModel(joint_model),
            lambda c: c.setIndexes(1, 2, 3),
            lambda c: c.id,
        )

        composite = pin.JointModelComposite(pin.JointModelRX())
        self.assertCopies(
            composite, lambda c: c.addJoint(pin.JointModelRY()), lambda c: c.njoints
        )

        # Joint datas are only read from Python: check the copy is an equal object.
        joint_data = pin.JointModelRX().createData()
        for route, copy_fn in COPY_ROUTES.items():
            with self.subTest(route=route):
                other = copy_fn(joint_data)
                self.assertIs(type(other), type(joint_data))
                self.assertTrue(other == joint_data)

    def test_std_vector(self):
        placements = pin.StdVec_SE3()
        placements.append(pin.SE3.Identity())
        self.assertCopies(
            placements,
            lambda c: c.__setitem__(0, pin.SE3.Random()),
            lambda c: c[0].homogeneous,
        )

    @unittest.skipUnless(coal_found, "needs coal")
    def test_geometry(self):
        """copy() and copy.copy share the coal geometry, copy.deepcopy clones it."""
        geom_object = pin.GeometryObject(
            "box", self.joint_id, pin.SE3.Identity(), coal.Box(1.0, 1.0, 1.0)
        )
        geom_model = pin.GeometryModel()
        geom_model.addGeometryObject(geom_object)

        def objectBox(obj):
            return obj.geometry

        def modelBox(model):
            return model.geometryObjects[0].geometry

        for obj, box in ((geom_object, objectBox), (geom_model, modelBox)):
            for route, copy_fn in COPY_ROUTES.items():
                with self.subTest(cls=type(obj).__name__, route=route):
                    other = copy_fn(obj)
                    self.assertIs(type(other), type(obj))
                    self.assertTrue(other == obj)
                    box(other).halfSide = np.full(3, 2.0)
                    shared = route != "copy.deepcopy"
                    self.assertEqual(
                        sameValue(box(obj).halfSide, np.full(3, 2.0)), shared
                    )
                    box(obj).halfSide = np.full(3, 0.5)

    def test_delassus_dense(self):
        self.assertCopies(
            pin.DelassusOperatorDense(np.eye(3)),
            lambda c: c.updateCompliance(1.0),
            lambda c: c.getCompliance(),
        )

    def test_graph(self):
        self.assertCopies(
            pin.graph.JointRevolute(np.array([1.0, 0.0, 0.0])),
            lambda c: setattr(c, "axis", np.array([0.0, 0.0, 1.0])),
            lambda c: c.axis,
        )

        graph = pin.graph.ModelGraph()
        graph.addBody("body1", pin.Inertia.Identity())

        def addBody(g):
            g.addBody("body2", pin.Inertia.Identity())
            g.edgeBuilder().withName("joint").withJointType(
                pin.graph.JointFreeFlyer()
            ).withSourceVertex("body1").withTargetVertex("body2").build()

        self.assertCopies(
            graph,
            addBody,
            lambda g: pin.graph.buildModel(g, "body1", pin.SE3.Identity()).njoints,
        )


if __name__ == "__main__":
    unittest.main()
