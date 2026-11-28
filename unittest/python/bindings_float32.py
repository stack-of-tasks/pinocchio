#
# Copyright (c) 2026 Heriot-Watt University
#

import importlib
import subprocess
import sys
import unittest

import numpy as np
import pinocchio as pin
import pinocchio.float32 as pin32


class TestFloat32Bindings(unittest.TestCase):
    def test_sample_models(self):
        for factory in (
            pin32.buildSampleModelManipulator,
            pin32.buildSampleModelHumanoid,
            pin32.buildSampleModelHumanoidRandom,
        ):
            model = factory()
            self.assertIsInstance(model, pin32.Model)
            self.assertNotIsInstance(model, pin.Model)
            self.assertEqual(pin32.neutral(model).dtype, np.float32)

    @unittest.skipUnless(pin32.WITH_COLLISION, "Needs collision support")
    def test_sample_geometry_models(self):
        for model_factory, geometry_factory in (
            (
                pin32.buildSampleModelManipulator,
                pin32.buildSampleGeometryModelManipulator,
            ),
            (
                pin32.buildSampleModelHumanoid,
                pin32.buildSampleGeometryModelHumanoid,
            ),
        ):
            geometry_model = geometry_factory(model_factory())
            self.assertIsInstance(geometry_model, pin.GeometryModel)
            self.assertGreater(geometry_model.ngeoms, 0)

    def test_rnea(self):
        self.assertIs(pin32.ScalarType, np.float32)
        self.assertIsNot(pin32.Model, pin.Model)

        model = pin.buildSampleModelHumanoidRandom()
        model32 = pin32.Model(model)
        data = model.createData()
        data32 = model32.createData()

        q = pin.neutral(model)
        v = np.linspace(-0.5, 0.5, model.nv)
        a = np.linspace(0.5, -0.5, model.nv)
        q32 = q.astype(np.float32)
        v32 = v.astype(np.float32)
        a32 = a.astype(np.float32)

        tau = pin.rnea(model, data, q, v, a)
        tau32 = pin32.rnea(model32, data32, q32, v32, a32)

        self.assertEqual(tau32.dtype, np.float32)
        np.testing.assert_allclose(tau32, tau, rtol=1e-5, atol=1e-5)

    def test_model_algorithms(self):
        model = pin32.buildSampleModelManipulator()
        q = pin32.neutral(model)

        appended = pin32.appendModel(model, pin32.Model(), 0, pin32.SE3.Identity())
        reduced = pin32.buildReducedModel(model, [1], q)
        mimic = pin32.transformJointIntoMimic(model, 1, 2, 1.0, 0.0)
        appended_with_geometry = pin32.appendModel(
            model,
            pin32.Model(),
            pin.GeometryModel(),
            pin.GeometryModel(),
            0,
            pin32.SE3.Identity(),
        )
        reduced_with_geometry = pin32.buildReducedModel(
            model, pin.GeometryModel(), [1], q
        )

        self.assertIsInstance(appended, pin32.Model)
        self.assertIsInstance(reduced, pin32.Model)
        self.assertIsInstance(mimic, pin32.Model)
        self.assertIsInstance(appended_with_geometry[0], pin32.Model)
        self.assertIsInstance(appended_with_geometry[1], pin.GeometryModel)
        self.assertIsInstance(reduced_with_geometry[0], pin32.Model)
        self.assertIsInstance(reduced_with_geometry[1], pin.GeometryModel)

    def test_supported_frame_quantities(self):
        model = pin32.buildSampleModelManipulator()
        data = model.createData()
        q = pin32.neutral(model)
        v = np.zeros(model.nv, dtype=np.float32)
        a = np.zeros(model.nv, dtype=np.float32)
        frame_id = model.getFrameId("effector_body")

        pin32.forwardKinematics(model, data, q)
        inertia = pin32.computeSupportedInertiaByFrame(model, data, frame_id, True)
        pin32.rnea(model, data, q, v, a)
        force = pin32.computeSupportedForceByFrame(model, data, frame_id)

        self.assertIsInstance(inertia, pin32.Inertia)
        self.assertIsInstance(force, pin32.Force)
        self.assertEqual(inertia.matrix().dtype, np.float32)
        self.assertEqual(force.vector.dtype, np.float32)

    def test_python_helpers(self):
        vector3 = np.ones(3, dtype=np.float32)
        matrix3 = np.eye(3, dtype=np.float32)

        self.assertEqual(pin32.exp(vector3).dtype, np.float32)
        self.assertEqual(pin32.log(matrix3).dtype, np.float32)
        self.assertIsInstance(pin32.exp(pin32.Motion.Zero()), pin32.SE3)
        self.assertIsInstance(pin32.log(pin32.SE3.Identity()), pin32.Motion)

        self.assertEqual(pin32.utils.eye(3).dtype, np.float32)
        self.assertEqual(pin32.utils.zero(3).dtype, np.float32)
        self.assertEqual(pin32.utils.rand(3).dtype, np.float32)
        self.assertEqual(pin32.utils.rpyToMatrix(vector3).dtype, np.float32)

    def test_submodule_imports(self):
        for module_name in (
            "cholesky",
            "liegroups",
            "linalg",
            "rpy",
            "serialization",
            "utils",
            "explog",
        ):
            module = importlib.import_module(f"pinocchio.float32.{module_name}")
            self.assertIsNotNone(module)

    def test_import_has_no_duplicate_converter_warning(self):
        process = subprocess.run(
            [sys.executable, "-c", "import pinocchio.float32"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertNotIn("converter already registered", process.stderr)

    def test_lcaba(self):
        model = pin32.buildSampleModelManipulator()
        data = model.createData()
        constraint_model = pin32.RigidConstraintModel(
            pin32.ContactType.CONTACT_3D,
            model,
            model.njoints - 1,
            pin32.SE3.Identity(),
            0,
            pin32.SE3.Identity(),
        )
        constraint_models = pin32.StdVec_RigidConstraintModel()
        constraint_models.append(constraint_model)
        constraint_datas = pin32.StdVec_RigidConstraintData()
        constraint_datas.append(constraint_model.createData())

        pin32.computeJointMinimalOrdering(model, data, constraint_models)
        ddq = pin32.lcaba(
            model,
            data,
            pin32.neutral(model),
            np.zeros(model.nv, dtype=np.float32),
            np.zeros(model.nv, dtype=np.float32),
            constraint_models,
            constraint_datas,
            pin32.ProximalSettings(1e-5, 1e-4, 2),
        )

        self.assertEqual(ddq.dtype, np.float32)

    def test_admm(self):
        model = pin32.Model()
        joint_id = model.addJoint(
            0, pin32.JointModelFreeFlyer(), pin32.SE3.Identity(), "free_flyer"
        )
        model.appendBodyToJoint(
            joint_id,
            pin32.Inertia.FromBox(1e-3, 1.0, 1.0, 1.0),
            pin32.SE3.Identity(),
        )

        rotation = np.eye(3, dtype=np.float32)
        upper = pin32.SE3(rotation, np.array([0.5, 0.5, 0.5], dtype=np.float32))
        lower = pin32.SE3(rotation, np.array([0.5, 0.5, -0.5], dtype=np.float32))
        point_contact = pin32.PointContactConstraintModel(
            model, 0, upper, joint_id, lower
        )
        point_contact.set = pin32.CoulombFrictionCone(0.4)
        constraint_models = pin32.StdVec_ConstraintModel()
        constraint_models.append(pin32.ConstraintModel(point_contact))
        constraint_datas = pin32.StdVec_ConstraintData()
        constraint_datas.append(constraint_models[0].createData())

        data = model.createData()
        q = pin32.neutral(model)
        zero = np.zeros(model.nv, dtype=np.float32)
        external_forces = pin32.StdVec_Force()
        external_forces.extend(pin32.Force.Zero() for _ in range(model.njoints))
        pin32.aba(model, data, q, zero, zero, external_forces)
        data.q_in = q
        data.v_in = zero
        data.tau_in = zero
        pin32.crba(model, data, q, pin32.Convention.WORLD)
        free_velocity = zero + np.float32(1e-3) * pin32.aba(
            model, data, q, zero, zero, external_forces
        )
        constraint_models[0].calc(model, data, constraint_datas[0])

        decomposition = pin32.ConstraintCholeskyDecomposition(
            model, data, constraint_models, constraint_datas
        )
        decomposition.compute(model, data, constraint_models, constraint_datas, 1e-6)
        delassus_matrix = decomposition.getDelassusOperatorCholeskyExpression().matrix()
        jacobian = pin32.getConstraintsJacobian(
            model, data, constraint_models, constraint_datas
        )
        drift = jacobian @ free_velocity

        settings = pin32.ADMMSolverSettings()
        settings.lanczos_size = drift.size
        settings.max_iterations = 2
        result = pin32.ADMMSolverResult()
        solver = pin32.ADMMConstraintSolver(drift.size)
        solver.solve(
            pin32.DelassusOperatorDense(delassus_matrix),
            drift,
            constraint_models,
            constraint_datas,
            settings,
            result,
        )

        self.assertEqual(delassus_matrix.dtype, np.float32)
        self.assertEqual(drift.dtype, np.float32)
        self.assertEqual(result.retrieveConstraintImpulses().dtype, np.float32)


if __name__ == "__main__":
    unittest.main()
