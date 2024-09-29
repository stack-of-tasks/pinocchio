import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


@unittest.skipUnless(pin.WITH_URDFDOM, "Needs URDFDOM")
class TestContactInverseDynamics(TestCase):
    def setUp(self):
        self.current_dir = Path(__file__).parent
        self.model_dir = self.current_dir / "../../models"
        self.model_path = self.model_dir / "example-robot-data/robots/talos_data"
        self.urdf_filename = "talos_reduced.urdf"
        self.srdf_filename = "talos.srdf"
        self.urdf_model_path = self.model_path / "robots" / self.urdf_filename
        self.srdf_full_path = self.model_path / "srdf" / self.srdf_filename

    def load_model(self):
        self.model = pin.buildModelFromUrdf(
            self.urdf_model_path, pin.JointModelFreeFlyer(), False
        )
        pin.loadReferenceConfigurations(self.model, self.srdf_full_path)
        self.q0 = self.model.referenceConfigurations["half_sitting"]

    def test_call_to_contact_inverse_dynamics(self):
        self.load_model()
        model = self.model
        feet_name = ["left_sole_link", "right_sole_link"]
        frame_ids = [model.getFrameId(frame_name) for frame_name in feet_name]

        q = self.q0
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        data = model.createData()

        contact_models_list = []
        contact_datas_list = []
        cones_list = []

        contact_models_vec = pin.StdVec_RigidConstraintModel()
        contact_datas_vec = pin.StdVec_RigidConstraintData()
        cones_vec = pin.StdVec_CoulombFrictionCone()

        for frame_id in frame_ids:
            frame = model.frames[frame_id]
            contact_model = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_3D, model, frame.parentJoint, frame.placement
            )

            contact_models_list.append(contact_model)
            contact_datas_list.append(contact_model.createData())
            cones_list.append(pin.CoulombFrictionCone(0.4))

            contact_models_vec.append(contact_model)
            contact_datas_vec.append(contact_model.createData())
            cones_vec.append(pin.CoulombFrictionCone(0.4))

        constraint_dim = 0
        for m in contact_models_list:
            constraint_dim += m.size()

        dt = 1e-3
        R = np.zeros(constraint_dim)
        constraint_correction = np.zeros(constraint_dim)
        lambda_guess = np.zeros(constraint_dim)
        prox_settings = pin.ProximalSettings(1e-12, 1e-6, 1)
        # pin.initConstraintDynamics(model, data, contact_models_list)  # not needed

        # test 1 with vector of contact models, contact datas and cones
        tau1 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_vec,
            contact_datas_vec,
            cones_vec,
            R,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )

        # test 2 with list of contact models, cones
        tau2 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_list,
            contact_datas_vec,
            cones_list,
            R,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )

        # test 3 with list of contact models, contact datas and cones
        tau3 = pin.contactInverseDynamics(
            model,
            data,
            q,
            v,
            a,
            dt,
            contact_models_list,
            contact_datas_list,
            cones_list,
            R,
            constraint_correction,
            prox_settings,
            lambda_guess,
        )
        self.assertApprox(tau1, tau2)
        self.assertApprox(tau1, tau3)


if __name__ == "__main__":
    unittest.main()
