import unittest
from pathlib import Path

import pinocchio as pin


class TestMjcfBindings(unittest.TestCase):
    def setUp(self):
        self.model_path = Path(__file__).parent.parent / "models" / "closed_chain.xml"

    def test_build_model_from_mjcf_returns_contacts_no_root_joint(self):
        model, contact_models = pin.buildModelFromMJCF(self.model_path)

        self.assertTrue(isinstance(model, pin.Model))
        self.assertEqual(len(contact_models), 4)

    def test_build_model_from_mjcf_returns_contacts_with_root_joint(self):
        model, contact_models = pin.buildModelFromMJCF(
            self.model_path, pin.JointModelFreeFlyer()
        )

        self.assertTrue(isinstance(model, pin.Model))
        self.assertEqual(len(contact_models), 4)


if __name__ == "__main__":
    unittest.main()
