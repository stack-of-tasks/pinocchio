import os
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestContactInverseDynamics(TestCase):
    def setUp(self):
        self.model_dir = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
        self.model_path = self.model_dir / "talos_data"
        self.urdf_filename = "talos_reduced.urdf"
        self.srdf_filename = "talos.srdf"
        self.urdf_model_path = self.model_path / "robots" / self.urdf_filename
        self.srdf_full_path = self.model_path / "srdf" / self.srdf_filename

    def load_model(self):
        self.model = pin.buildModelFromUrdf(
            self.urdf_model_path, pin.JointModelFreeFlyer()
        )
        pin.loadReferenceConfigurations(self.model, self.srdf_full_path)
        self.q0 = self.model.referenceConfigurations["half_sitting"]

    def test_call_to_contact_inverse_dynamics(self):
        self.load_model()
        model = self.model
        feet_name = ["left_sole_link", "right_sole_link"]
        frame_ids = [model.getFrameId(frame_name) for frame_name in feet_name]

        q = self.q0
        v = 0 * np.random.rand(model.nv)
        a = 0 * np.random.rand(model.nv)
        data = model.createData()

        contact_models_vec = pin.StdVec_PointContactConstraintModel()
        contact_datas_vec = pin.StdVec_PointContactConstraintData()

        for frame_id in frame_ids:
            frame = model.frames[frame_id]
            contact_model = pin.PointContactConstraintModel(
                model, frame.parentJoint, frame.placement
            )
            contact_model.setFriction(0.4)

            contact_models_vec.append(contact_model)
            contact_datas_vec.append(contact_model.createData())

        constraint_size = 0
        for cm, cd in zip(contact_models_vec, contact_datas_vec):
            constraint_size += cm.residualSize()

        dt = 1e-3
        constraint_correction = np.zeros(constraint_size)
        lambda_guess = np.zeros(constraint_size)
        prox_settings = pin.ProximalSettings(1e-12, 1e-6, 1)

        # Compute all the Jacobians and call cacl on each constraint
        pin.computeJointJacobians(model, data, q)
        for cm, cd in zip(contact_models_vec, contact_datas_vec):
            cm.calc(model, data, cd)

        # test 1 with vector of contact models, contact datas and cones
        has_converged1, tau_sol1, _lambda_sol1 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_vec,
            contact_datas_vec,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )
        self.assertEqual(has_converged1, True)

        # test 2 with list of contact models, cones
        has_converged2, tau_sol2, _lambda_sol2 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_vec,
            contact_datas_vec,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )
        self.assertEqual(has_converged2, True)

        # test 3 with list of contact models, contact datas and cones
        has_converged3, tau_sol3, _lambda_sol3 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_vec,
            contact_datas_vec,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )
        self.assertEqual(has_converged3, True)

        self.assertApprox(tau_sol1, tau_sol2)
        self.assertApprox(tau_sol2, tau_sol3)
        self.assertApprox(tau_sol3, tau_sol1)


if __name__ == "__main__":
    unittest.main()
