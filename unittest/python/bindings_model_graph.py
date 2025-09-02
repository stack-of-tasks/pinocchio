import unittest

import numpy as np
import pinocchio as pin


class TestModelGraphBindings(unittest.TestCase):
    def test_build_model(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())

        g.edgeBuilder().withName("b1_b2").withJointType(
            pin.graph.JointPrismatic(np.array([1, 0, 0]))
        ).withSourceVertex("body1").withSourcePose(pin.SE3.Random()).withTargetVertex(
            "body2"
        ).withTargetPose(pin.SE3.Random()).build()

        m = pin.graph.buildModel(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

        m, _ = pin.graph.buildModelWithBuildInfo(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

    def test_merge(self):
        g1 = pin.graph.ModelGraph()
        g1.addBody("body1", pin.Inertia.Identity())
        g2 = pin.graph.ModelGraph()
        g2.addBody("body2", pin.Inertia.Identity())
        g = pin.graph.merge(
            g1,
            g2,
            "body1",
            "body2",
            pin.SE3.Random(),
            pin.graph.JointPrismatic(np.array([1, 0, 0])),
            "b1_b2",
        )
        m = pin.graph.buildModel(g, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "b1_b2")

    def test_lock_joints(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())

        g.edgeBuilder().withName("b1_b2").withJointType(
            pin.graph.JointPrismatic(np.array([1, 0, 0]))
        ).withSourceVertex("body1").withSourcePose(pin.SE3.Random()).withTargetVertex(
            "body2"
        ).withTargetPose(pin.SE3.Random()).build()

        g_lock = pin.graph.lockJoints(g, ["b1_b2"], [np.array([0.3])])
        m = pin.graph.buildModel(g_lock, "body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 1)

    def test_prefix_names(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())

        g.edgeBuilder().withName("b1_b2").withJointType(
            pin.graph.JointPrismatic(np.array([1, 0, 0]))
        ).withSourceVertex("body1").withSourcePose(pin.SE3.Random()).withTargetVertex(
            "body2"
        ).withTargetPose(pin.SE3.Random()).build()

        g1 = pin.graph.prefixNames(g, "g1/")
        m = pin.graph.buildModel(g1, "g1/body1", pin.SE3.Identity())
        self.assertTrue(m.njoints == 2)
        self.assertTrue(m.names[1] == "g1/b1_b2")

    def test_converter(self):
        g = pin.graph.ModelGraph()
        g.addBody("body1", pin.Inertia.Identity())
        g.addBody("body2", pin.Inertia.Identity())
        g.addBody("body3", pin.Inertia.Identity())

        g.edgeBuilder().withName("b1_b2").withJointType(
            pin.graph.JointPrismatic(np.array([1, 0, 0]))
        ).withSourceVertex("body1").withSourcePose(pin.SE3.Random()).withTargetVertex(
            "body2"
        ).withTargetPose(pin.SE3.Random()).build()

        g.edgeBuilder().withName("b2_b3").withJointType(
            pin.graph.JointPrismatic(np.array([0, 1, 0]))
        ).withSourceVertex("body2").withSourcePose(pin.SE3.Random()).withTargetVertex(
            "body3"
        ).withTargetPose(pin.SE3.Random()).build()

        m_body1, build_info_body1 = pin.graph.buildModelWithBuildInfo(
            g, "body1", pin.SE3.Identity()
        )
        q_body1 = pin.randomConfiguration(
            m_body1, np.array([-1.0] * 2), np.array([1.0] * 2)
        )
        v_body1 = np.random.random(m_body1.nv)

        m_body3, build_info_body3 = pin.graph.buildModelWithBuildInfo(
            g, "body3", pin.SE3.Identity()
        )
        converter = pin.graph.createConverter(
            m_body1, m_body3, build_info_body1, build_info_body3
        )
        q_body3 = converter.convertConfigurationVector(q_body1)
        self.assertTrue(q_body1[0] == -q_body3[-1])
        self.assertTrue(q_body1[1] == -q_body3[-2])

        v_body3 = converter.convertTangentVector(q_body1, v_body1)
        self.assertTrue(v_body1[0] == -v_body3[-1])
        self.assertTrue(v_body1[1] == -v_body3[-2])


if __name__ == "__main__":
    unittest.main()
