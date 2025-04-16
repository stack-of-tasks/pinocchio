import unittest

import pinocchio as pin
from test_case import PinocchioTestCase as TestCase
from test_ext_bindings_visualizer import DummyVisualizer


class TestBindingsViz(TestCase):
    def setUp(self):
        self.model = pin.Model()
        self.visual = pin.GeometryModel()
        self.viz = DummyVisualizer(self.model, self.visual)

    def test_getters(self):
        self.assertEqual(self.model, self.viz.model)
        self.assertEqual(self.visual, self.viz.visualModel)


if __name__ == "__main__":
    unittest.main()
