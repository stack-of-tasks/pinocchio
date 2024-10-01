#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest

import pinocchio as pin


class TestRobotWrapper(unittest.TestCase):
    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))

    def test_mjcf_without_root_joint(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/test_mjcf.xml")
        )
        robot = pin.RobotWrapper.BuildFromMJCF(model_path)
        self.assertEqual(robot.nq, 6)
        self.assertEqual(robot.nv, 5)
        self.assertEqual(robot.model.njoints, 4)

    def test_mjcf_with_root_joint(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/test_mjcf.xml")
        )
        robot = pin.RobotWrapper.BuildFromMJCF(model_path, pin.JointModelFreeFlyer())
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_mjcf_with_root_joint_and_root_joint_name(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/test_mjcf.xml")
        )
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromMJCF(
            model_path, pin.JointModelFreeFlyer(), name_
        )
        self.assertEqual(robot.model.names[1], name_)

    def test_urdf_with_root_joint(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/3DOF_planar.urdf")
        )
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [], pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_root_joint_and_root_joint_name(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/3DOF_planar.urdf")
        )
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [], pin.JointModelFreeFlyer(), name_
        )
        self.assertEqual(robot.model.names[1], name_)

    @unittest.skipUnless(pin.WITH_SDFORMAT, "Needs SDFORMAT")
    def test_sdf_with_root_joint(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../../models/simple_humanoid.sdf")
        )
        mesh_path = os.path.abspath(os.path.join(self.current_file, "../../models/"))
        robot = pin.RobotWrapper.BuildFromSDF(
            model_path, [mesh_path], pin.JointModelFreeFlyer(), verbose=True
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    @unittest.skipUnless(pin.WITH_SDFORMAT, "Needs SDFORMAT")
    def test_sdf_with_root_joint_and_root_joint_name(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../../models/simple_humanoid.sdf")
        )
        mesh_path = os.path.abspath(os.path.join(self.current_file, "../../models/"))
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromSDF(
            model_path, [mesh_path], pin.JointModelFreeFlyer(), root_joint_name=name_
        )
        self.assertEqual(robot.model.names[1], name_)


if __name__ == "__main__":
    unittest.main()
