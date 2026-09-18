import importlib.util
import unittest
from pathlib import Path

import pinocchio as pin

mujoco_spec = importlib.util.find_spec("mujoco")
mujoco_found = mujoco_spec is not None


class TestMJCFBindings(unittest.TestCase):
    def test_load(self):
        current_dir = Path(__file__).parent
        model_dir = current_dir / "../models/"
        model_path = model_dir / "closed_chain.xml"
        _, point_anchor_constraint_models, _ = pin.buildModelAndConstraintsFromMJCF(
            model_path
        )
        self.assertEqual(len(point_anchor_constraint_models), 4)

    def test_build_model_from_content(self):
        model_path = Path(__file__).parent / "../../models/simple_humanoid.xml"
        mjcf_string = model_path.read_text()

        model = pin.buildModelFromMJCFContent(mjcf_string)
        self.assertEqual(model, pin.buildModelFromMJCF(model_path))

        root_joint = pin.JointModelFreeFlyer()
        model_root_joint = pin.buildModelFromMJCFContent(mjcf_string, root_joint)
        self.assertEqual(model_root_joint.names[1], "root_joint")
        self.assertEqual(
            model_root_joint,
            pin.buildModelFromMJCFAndRootJoint(model_path, root_joint),
        )

        model_root_joint_name = pin.buildModelFromMJCFContent(
            mjcf_string, root_joint, "floating"
        )
        self.assertEqual(model_root_joint_name.names[1], "floating")

    @unittest.skipUnless(pin.WITH_COLLISION, "Needs collision support")
    def test_build_geom_from_content(self):
        model_path = Path(__file__).parent / "../models/closed_chain.xml"
        mjcf_string = model_path.read_text()

        model = pin.buildModelFromMJCFContent(mjcf_string)
        geom_model = pin.buildGeomFromMJCF(
            model, model_path, pin.GeometryType.COLLISION
        )
        geom_model_content = pin.buildGeomFromMJCFContent(
            model, mjcf_string, pin.GeometryType.COLLISION
        )
        self.assertGreater(len(geom_model_content.geometryObjects), 0)
        self.assertEqual(geom_model_content, geom_model)


@unittest.skipUnless(mujoco_found, "Needs MuJoCo.")
class TestMJCFBindingsWithMujoco(unittest.TestCase):
    def test_cassie(self):
        import mujoco
        import numpy as np

        current_dir = Path(__file__).parent
        model_dir = current_dir / "../models/"
        model_path = model_dir / "closed_chain.xml"
        (
            model_pin,
            point_anchor_constraint_models_pin,
            frame_anchor_constraint_models_pin,
        ) = pin.buildModelAndConstraintsFromMJCF(model_path)
        print(point_anchor_constraint_models_pin, frame_anchor_constraint_models_pin)
        data_pin = model_pin.createData()
        q0_pin = model_pin.referenceConfigurations["home"]
        pin.forwardKinematics(model_pin, data_pin, q0_pin)
        model_mj = mujoco.MjModel.from_xml_path(str(model_path))
        data_mj = mujoco.MjData(model_mj)
        q0_mujoco = model_mj.key_qpos[0].copy()
        data_mj.qpos = q0_mujoco
        mujoco.mj_fwdPosition(model_mj, data_mj)
        for joint_id in range(model_mj.njnt):  # Total number of joints
            assert (
                np.linalg.norm(
                    data_mj.xanchor[joint_id] - data_pin.oMi[joint_id + 1].translation
                )
                < 1e-6
            )
        return


if __name__ == "__main__":
    unittest.main()
