#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: BSD-2-Clause

import os
import unittest

import pinocchio as pin


class TestRobotWrapper(unittest.TestCase):
    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))

    def test_mjcf(self):
        model_path = os.path.abspath(
            os.path.join(self.current_file, "../models/test_mjcf.xml")
        )
        robot = pin.RobotWrapper.BuildFromMJCF(model_path)
        self.assertEqual(robot.nq, 6)
        self.assertEqual(robot.nv, 5)
        self.assertEqual(robot.model.njoints, 4)


if __name__ == "__main__":
    unittest.main()
