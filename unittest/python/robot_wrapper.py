#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause

import unittest
from pathlib import Path

import pinocchio as pin


class TestRobotWrapper(unittest.TestCase):
    def setUp(self):
        self.current_dir = Path(__file__).parent

    def test_mjcf_without_root_joint(self):
        model_path = self.current_dir.parent / "models" / "test_mjcf.xml"
        robot = pin.RobotWrapper.BuildFromMJCF(model_path)
        self.assertEqual(robot.nq, 6)
        self.assertEqual(robot.nv, 5)
        self.assertEqual(robot.model.njoints, 4)

    def test_mjcf_with_root_joint(self):
        model_path = self.current_dir.parent / "models" / "test_mjcf.xml"
        robot = pin.RobotWrapper.BuildFromMJCF(model_path, pin.JointModelFreeFlyer())
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_mjcf_with_root_joint_and_root_joint_name(self):
        model_path = self.current_dir.parent / "models" / "test_mjcf.xml"
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromMJCF(
            model_path, pin.JointModelFreeFlyer(), name_
        )
        self.assertEqual(robot.model.names[1], name_)

    def test_urdf_with_root_joint(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [], pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_root_joint_and_root_joint_name(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [], pin.JointModelFreeFlyer(), name_
        )
        self.assertEqual(robot.model.names[1], name_)

    def test_urdf_with_str_pkg_dirs(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        package_dir = self.current_dir.parent / "models"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, str(package_dir), pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_path_pkg_dirs(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        package_dir = self.current_dir.parent / "models"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, package_dir, pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_str_list_pkg_dirs(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        package_dir = self.current_dir.parent / "models"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [str(package_dir)], pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_path_list_pkg_dirs(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        package_dir = self.current_dir.parent / "models"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, [package_dir], pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    def test_urdf_with_None_pkg_dirs(self):
        model_path = self.current_dir.parent / "models" / "3DOF_planar.urdf"
        robot = pin.RobotWrapper.BuildFromURDF(
            model_path, None, pin.JointModelFreeFlyer()
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    @unittest.skipUnless(pin.WITH_SDFORMAT, "Needs SDFORMAT")
    def test_sdf_with_root_joint(self):
        model_path = self.current_dir.parent.parent / "models" / "simple_humanoid.sdf"
        mesh_path = self.current_dir.parent.parent / "models"
        robot = pin.RobotWrapper.BuildFromSDF(
            model_path, [mesh_path], pin.JointModelFreeFlyer(), verbose=True
        )
        self.assertEqual(robot.model.names[1], "root_joint")

    @unittest.skipUnless(pin.WITH_SDFORMAT, "Needs SDFORMAT")
    def test_sdf_with_root_joint_and_root_joint_name(self):
        model_path = self.current_dir.parent.parent / "models" / "simple_humanoid.sdf"
        mesh_path = self.current_dir.parent.parent / "models"
        name_ = "freeflyer_joint"
        robot = pin.RobotWrapper.BuildFromSDF(
            model_path, [mesh_path], pin.JointModelFreeFlyer(), root_joint_name=name_
        )
        self.assertEqual(robot.model.names[1], name_)

    # Non regression test for this issue: https://github.com/stack-of-tasks/pinocchio/issues/2980
    # Following 4 tests cover all cases:
    # - No geometry
    # - Only visual
    # - Only collision
    # - Both
    def test_build_reduced_model_no_geoms(self):
        robot = pin.RobotWrapper(model=pin.buildSampleModelManipulator())
        reduced_robot = robot.buildReducedRobot([2])
        assert reduced_robot.visual_model is None
        assert reduced_robot.collision_model is None

    @unittest.skipUnless(pin.WITH_COLLISION, "Needs collision support")
    def test_build_reduced_model_visual_geoms(self):
        model = pin.buildSampleModelManipulator()
        robot = pin.RobotWrapper(
            model=model,
            visual_model=pin.buildSampleGeometryModelManipulator(model),
        )
        reduced_robot = robot.buildReducedRobot([2])
        assert reduced_robot.visual_model is not None
        assert reduced_robot.collision_model is None

    @unittest.skipUnless(pin.WITH_COLLISION, "Needs collision support")
    def test_build_reduced_model_collision_geoms(self):
        model = pin.buildSampleModelManipulator()
        robot = pin.RobotWrapper(
            model=model,
            collision_model=pin.buildSampleGeometryModelManipulator(model),
        )
        reduced_robot = robot.buildReducedRobot([2])
        assert reduced_robot.visual_model is None
        assert reduced_robot.collision_model is not None

    @unittest.skipUnless(pin.WITH_COLLISION, "Needs collision support")
    def test_build_reduced_model_all_geoms(self):
        model = pin.buildSampleModelManipulator()
        robot = pin.RobotWrapper(
            model=model,
            visual_model=pin.buildSampleGeometryModelManipulator(model),
            collision_model=pin.buildSampleGeometryModelManipulator(model),
        )
        # Modify collision_model to check they are not mixed-up
        robot.collision_model.addAllCollisionPairs()

        reduced_robot = robot.buildReducedRobot([2])
        assert reduced_robot.visual_model is not None
        assert len(reduced_robot.visual_model.collisionPairs) == 0
        assert reduced_robot.collision_model is not None
        assert len(reduced_robot.collision_model.collisionPairs) > 0


if __name__ == "__main__":
    unittest.main()
