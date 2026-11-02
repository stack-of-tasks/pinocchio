import itertools
import unittest

import coal as coal
import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestJointsAlgo(TestCase):
    def setUp(self):
        self.construct_model()
        self.construct_constraints()

    def construct_model(self):
        model = pin.Model()
        geom_model = pin.GeometryModel()

        world_joint_id = 0

        # Add a plane in world model
        plane_geom_name = "geom_plane"
        plane_geom_shape = coal.Halfspace(0.0, 0.0, 1.0, 0.0)
        plane_geom_placement = pin.SE3.Identity()
        plane_geom_obj = pin.GeometryObject(
            plane_geom_name, world_joint_id, plane_geom_placement, plane_geom_shape
        )
        plane_geom_id = geom_model.addGeometryObject(plane_geom_obj)

        # Add a freeflyer with a ball attach to it
        ball_joint_name = "joint_ball_ff"
        ball_joint_obj = pin.JointModelFreeFlyer()
        ball_joint_placement = pin.SE3.Identity()
        ball_joint_id = model.addJoint(
            world_joint_id, ball_joint_obj, ball_joint_placement, ball_joint_name
        )
        ball_geom_radius = 1.0
        ball_geom_name = "geom_ball_ff"
        ball_geom_shape = coal.Sphere(ball_geom_radius)
        ball_geom_placement = pin.SE3.Identity()
        ball_geom_obj = pin.GeometryObject(
            ball_geom_name, ball_joint_id, ball_geom_placement, ball_geom_shape
        )
        ball_geom_id = geom_model.addGeometryObject(ball_geom_obj)
        ball_body_mass = 1.0
        ball_body_placement = pin.SE3.Identity()
        ball_body_inertia = pin.Inertia.FromSphere(ball_body_mass, ball_geom_radius)
        model.appendBodyToJoint(
            ball_joint_id, ball_body_inertia, ball_body_placement
        )  # Add inertia of the ball to the model
        geom_model.addCollisionPair(
            pin.CollisionPair(plane_geom_id, ball_geom_id)
        )  # Add collision pair with the floor to the floor

        # Add a prismatic with a cylinder attach to it
        cyl_joint_name = "joint_cyl_PZ"
        cyl_joint_obj = pin.JointModelPZ()
        cyl_joint_placement = pin.SE3.Identity()
        cyl_joint_id = model.addJoint(
            world_joint_id, cyl_joint_obj, cyl_joint_placement, cyl_joint_name
        )
        cyl_geom_radius = 1.0
        cyl_geom_length = 1.0
        cyl_geom_name = "geom_cyl_PZ"
        cyl_geom_shape = coal.Cylinder(cyl_geom_radius, cyl_geom_length)
        cyl_geom_placement = pin.SE3.Identity()
        cyl_geom_obj = pin.GeometryObject(
            cyl_geom_name, cyl_joint_id, cyl_geom_placement, cyl_geom_shape
        )
        cyl_geom_id = geom_model.addGeometryObject(cyl_geom_obj)
        cyl_body_mass = 1.0
        cyl_body_placement = pin.SE3.Identity()
        cyl_body_inertia = pin.Inertia.FromCylinder(
            cyl_body_mass, cyl_geom_radius, cyl_geom_length
        )
        model.appendBodyToJoint(
            cyl_joint_id, cyl_body_inertia, cyl_body_placement
        )  # Add inertia of the ball to the model
        geom_model.addCollisionPair(
            pin.CollisionPair(plane_geom_id, cyl_geom_id)
        )  # Add collision pair with the floor to the floor

        # Add a revolute joint with a capsule attach to it
        caps_joint_name = "joint_caps_RX"
        caps_joint_obj = pin.JointModelRX()
        caps_joint_placement = pin.SE3.Identity()
        caps_joint_id = model.addJoint(
            world_joint_id, caps_joint_obj, caps_joint_placement, caps_joint_name
        )
        caps_geom_radius = 1.0
        caps_geom_halflength = 1.0
        caps_geom_name = "geom_caps_RX"
        caps_geom_shape = coal.Capsule(caps_geom_radius, caps_geom_halflength)
        caps_geom_placement = pin.SE3.Identity()
        caps_geom_obj = pin.GeometryObject(
            caps_geom_name, caps_joint_id, caps_geom_placement, caps_geom_shape
        )
        caps_geom_id = geom_model.addGeometryObject(caps_geom_obj)
        caps_body_mass = 1.0
        caps_body_placement = pin.SE3.Identity()
        caps_body_inertia = pin.Inertia.FromCylinder(
            caps_body_mass, caps_geom_radius, caps_geom_halflength * 2
        )
        model.appendBodyToJoint(
            caps_joint_id, caps_body_inertia, caps_body_placement
        )  # Add inertia of the ball to the model
        geom_model.addCollisionPair(pin.CollisionPair(plane_geom_id, caps_geom_id))
        geom_model.addCollisionPair(pin.CollisionPair(ball_geom_id, caps_geom_id))

        # Set up limits
        model.positionLimitMargin = np.array(
            [1000.0] * model.nq
        )  # take all joint limits into consideration
        model.upperPositionLimit = np.array([0.5] * model.nq)
        model.upperPositionLimit[-1] = 2.0
        model.lowerPositionLimit = np.array([-0.5] * model.nq)
        model.lowerDryFrictionLimit = -np.ones(model.nv)
        model.upperDryFrictionLimit = np.ones(model.nv)

        # Store some values
        self.model = model
        self.geom_model = geom_model
        self.world_joint_id = world_joint_id
        self.caps_joint_id = caps_joint_id
        self.cyl_joint_id = cyl_joint_id

    def construct_constraints(self):
        model = self.model
        geom_model = self.geom_model
        world_joint_id = self.world_joint_id
        caps_joint_id = self.caps_joint_id
        cyl_joint_id = self.cyl_joint_id

        # Add all constraints
        constraints_std_vec = pin.StdVec_ConstraintModel()
        constraints_list = []

        # Bilateral constraints
        bilat_joint1_anchor_placement = pin.SE3.Identity()
        bilat1 = pin.PointAnchorConstraintModel(model, caps_joint_id)
        bilat2 = pin.PointAnchorConstraintModel(
            model, caps_joint_id, bilat_joint1_anchor_placement
        )
        bilat3 = pin.PointAnchorConstraintModel(model, caps_joint_id, world_joint_id)
        bilat4 = pin.PointAnchorConstraintModel(
            model,
            caps_joint_id,
            bilat_joint1_anchor_placement,
            world_joint_id,
            pin.SE3.Identity(),
        )
        bilats_list = [bilat1, bilat2, bilat3, bilat4]
        for b in bilats_list:
            gb = pin.ConstraintModel(b)
            constraints_std_vec.append(gb)
            constraints_list.append(b)

        # Frame anchor constraints
        frame_anchor_joint1_placement = pin.SE3.Identity()
        facm1 = pin.FrameAnchorConstraintModel(model, cyl_joint_id)
        facm2 = pin.FrameAnchorConstraintModel(
            model, cyl_joint_id, frame_anchor_joint1_placement
        )
        facm3 = pin.FrameAnchorConstraintModel(model, cyl_joint_id, world_joint_id)
        facm4 = pin.FrameAnchorConstraintModel(
            model,
            cyl_joint_id,
            frame_anchor_joint1_placement,
            world_joint_id,
            pin.SE3.Identity(),
        )
        facms_list = [facm1, facm2, facm3, facm4]
        for w in facms_list:
            gw = pin.ConstraintModel(w)
            constraints_std_vec.append(gw)
            constraints_list.append(w)

        # Joint friction
        jfc = pin.JointFrictionConstraintModel(model, [1, 3])
        jfc.setFrictionLowerLimit(
            np.array([model.lowerDryFrictionLimit[i] for i in jfc.getActiveDofs()])
        )
        jfc.setFrictionUpperLimit(
            np.array([model.upperDryFrictionLimit[i] for i in jfc.getActiveDofs()])
        )
        constraints_std_vec.append(pin.ConstraintModel(jfc))
        constraints_list.append(jfc)

        # Limit joint
        jlc = pin.JointLimitConstraintModel(model, [1, 2, 3])
        constraints_std_vec.append(pin.ConstraintModel(jlc))
        constraints_list.append(jlc)

        # Create some contacts and active limits
        data = model.createData()
        geom_data = geom_model.createData()
        q = np.array(
            [
                -0.49999217,
                -0.36846221,
                0.25560532,
                -0.72022615,
                0.66439729,
                0.13124927,
                -0.1504133,
                -0.45295538,
                1.19716179,
            ]
        )
        self.q = q
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateGeometryPlacements(model, data, geom_model, geom_data)
        pin.computeCollisions(model, data, geom_model, geom_data, q, False)
        pin.computeContactPatches(geom_model, geom_data)
        data.q_in = q

        point_contact_list = []
        for col_pair, col_res, patch_res in zip(
            geom_model.collisionPairs,
            geom_data.collisionResults,
            geom_data.contactPatchResults,
        ):
            if col_res.isCollision() and patch_res.numContactPatches() > 0:
                geom_id1 = col_pair.first
                geom_id2 = col_pair.second
                geom1 = geom_model.geometryObjects[geom_id1]
                geom2 = geom_model.geometryObjects[geom_id2]
                joint_id1 = geom1.parentJoint
                joint_id2 = geom2.parentJoint
                oMi1 = data.oMi[joint_id1]
                oMi2 = data.oMi[joint_id2]
                patch = patch_res.getContactPatch(0)
                contact_normal = patch.getNormal()
                oMc = pin.SE3.Identity()
                contact_normal = contact_normal / np.linalg.norm(contact_normal)
                e_ref = (
                    np.array([0, 1, 0])
                    if np.isclose(contact_normal[0], 1.0)
                    else np.array([1, 0, 0])
                )
                comp = np.cross(e_ref, contact_normal)
                comp = comp / np.linalg.norm(comp)
                oMc.rotation[:, 2] = contact_normal
                oMc.rotation[:, 0] = comp
                oMc.rotation[:, 1] = np.cross(contact_normal, comp)
                for i in range(patch.size()):
                    oMc.translation = patch.getPointShape1(i)
                    i1Mc = oMi1.actInv(oMc)
                    oMc.translation = patch.getPointShape2(i)
                    i2Mc = oMi2.actInv(oMc)
                    fp = pin.PointContactConstraintModel(
                        model, joint_id1, i1Mc, joint_id2, i2Mc
                    )
                    gfp = pin.ConstraintModel(fp)
                    constraints_std_vec.append(gfp)
                    constraints_list.append(fp)
                    point_contact_list.append(fp)

        # Sotre some values
        self.data = data
        self.geom_data = geom_data
        self.constraints_std_vec = constraints_std_vec
        self.constraints_list = constraints_list
        self.bilats_list = bilats_list
        self.bilat = bilat1
        self.facms_list = facms_list
        self.facm = facm1
        self.point_contact_list = point_contact_list
        self.fp = point_contact_list[0]
        self.jlc = jlc
        self.jfc = jfc
        self.one_of_each = [self.bilat, self.facm, self.fp, self.jlc, self.jfc]

    def test_point_anchor(self):
        # Coherence between all inits
        for b_1, b_2 in itertools.product(self.bilats_list, self.bilats_list):
            self.assertTrue(b_1 == b_2)

    def test_frame_anchor(self):
        # Coherence between all inits
        for w_1, w_2 in itertools.product(self.facms_list, self.facms_list):
            self.assertTrue(w_1 == w_2)

    def test_point_contact(self):
        for fp in self.point_contact_list:
            fpd = fp.createData()
            assert fpd

    def test_constant_length(self):
        placement1 = pin.SE3(np.eye(3), np.array([0.1, 0.0, 0.0]))
        placement2 = pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.0]))
        cmodel = pin.ConstantLengthConstraintModel(
            self.model, 1, placement1, 0, placement2, 0.5
        )
        self.assertEqual(cmodel.getLength(), 0.5)
        cmodel.setLength(0.25)
        self.assertEqual(cmodel.getLength(), 0.25)

        q = pin.neutral(self.model)
        q[:3] = [0.3, -0.2, 0.4]
        pin.forwardKinematics(self.model, self.data, q)
        cdata = cmodel.createData()
        cmodel.calc(self.model, self.data, cdata)
        point1 = (self.data.oMi[1] * placement1).translation
        expected_distance = np.linalg.norm(placement2.translation - point1)
        self.assertAlmostEqual(cdata.distance, expected_distance)
        self.assertTrue(np.allclose(cdata.direction * cdata.distance, cdata.relative_position))

    def test_joint_friction(self):
        jfcd = self.jfc.createData()
        assert jfcd

    def test_joint_limit(self):
        jld = self.jlc.createData()
        assert jld
        self.assertTrue(self.jlc.residualSize() <= 2 * self.model.nq)
        self.assertTrue(
            self.jlc.residualSize()
            <= self.jlc.residualSize(pin.ConstraintSelectionType.MAXIMAL)
        )
        self.jlc.makeSelectionFilteredByLimitProximity(self.q)
        self.assertTrue(
            self.jlc.residualSize()
            <= self.jlc.residualSize(pin.ConstraintSelectionType.MAXIMAL)
        )
        self.jlc.makeSelectionMaximal()
        self.assertTrue(
            self.jlc.residualSize()
            == self.jlc.residualSize(pin.ConstraintSelectionType.MAXIMAL)
        )

    def test_generic_methods(self):
        ref_set = set(range(self.model.nv))

        for gcm, ccm in zip(self.constraints_std_vec, self.constraints_list):
            # Test hierarchy
            self.assertTrue(gcm.extract() == ccm)
            self.assertTrue(gcm.shortname() == ccm.classname() == ccm.shortname())
            gcd = gcm.createData()
            gcm.calc(self.model, self.data, gcd)
            ccd = ccm.createData()
            ccm.calc(self.model, self.data, ccd)
            for i in range(gcm.residualSize()):
                self.assertTrue(
                    np.all(
                        np.where(
                            gcm.getRowSparsityPattern(self.model, self.data, gcd, i)
                        )[0]
                        == np.array(gcm.getRowIndexes(self.model, self.data, gcd, i))
                    )
                )
                self.assertTrue(
                    set(gcm.getRowIndexes(self.model, self.data, gcd, i)) <= ref_set
                )
                self.assertTrue(
                    gcm.residualSize()
                    <= gcm.residualSize(pin.ConstraintSelectionType.MAXIMAL)
                )
            # default compliance should be 0
            self.assertTrue(
                np.all(np.zeros(gcm.residualSize()) == gcm.retrieveCompliance())
            )
            # test settings compliance
            dummy_compliance = 0.1 * np.ones(gcm.residualSize())
            gcm.setCompliance(dummy_compliance)
            self.assertTrue(np.all(dummy_compliance == gcm.retrieveCompliance()))
            ccm.setCompliance(dummy_compliance)
            self.assertTrue(np.all(dummy_compliance == ccm.retrieveCompliance()))
            self.assertTrue(len(ccm.retrieveCompliance()) == ccm.residualSize())
            if not hasattr(ccm, "baumgarte_corrector_parameters"):
                self.assertTrue(isinstance(ccm, pin.JointFrictionConstraintModel))
                do_except = False
                self.assertTrue("baumgarte_corrector_parameters" in dir(gcm))
                try:
                    gcm.baumgarte_corrector_parameters
                except Exception as _:
                    do_except = True
                self.assertTrue(do_except)
            else:
                bp = pin.BaumgarteCorrectorParameters(1.0, 2.0)
                ccm.setBaumgarteCorrectorParameters(bp)
                self.assertTrue(ccm.baumgarte_corrector_parameters.Kp == 1.0)
                self.assertTrue(ccm.baumgarte_corrector_parameters.Kd == 2.0)
                #
                bp = pin.BaumgarteCorrectorParameters(2.0, 3.0)
                gcm.setBaumgarteCorrectorParameters(bp)
                self.assertTrue(gcm.baumgarte_corrector_parameters.Kp == 2.0)
                self.assertTrue(gcm.baumgarte_corrector_parameters.Kd == 3.0)

    def test_jacobians_methods(self):
        model = self.model
        data = self.data

        v = np.stack([np.ones(model.nv), 2 * np.ones(model.nv)], axis=1)
        for cmodel in self.one_of_each:
            gcmodel = pin.ConstraintModel(cmodel)
            cdata = cmodel.createData()
            cmodel.calc(self.model, self.data, cdata)
            gcdata = gcmodel.createData()
            gcmodel.calc(self.model, self.data, gcdata)
            g2cdata = pin.ConstraintData(cdata)

            self.assertTrue(g2cdata == gcdata)
            self.assertTrue(gcdata.extract() == cdata)
            self.assertTrue(g2cdata.extract() == cdata)

            self.assertTrue(
                gcdata.shortname() == cdata.classname() == cdata.shortname()
            )

            lamb = np.stack(
                [
                    np.ones(cmodel.residualSize()),
                    2 * np.ones(cmodel.residualSize()),
                ],
                axis=1,
            )

            jac = cmodel.jacobian(model, data, cdata)
            sig = cmodel.jacobianMatrixProduct(model, data, cdata, v)
            tau = cmodel.jacobianTransposeMatrixProduct(model, data, cdata, lamb)

            self.assertTrue(np.all(np.isclose(jac @ v, sig)))
            self.assertTrue(np.all(np.isclose(jac.T @ lamb, tau)))

            if not isinstance(cmodel, pin.JointLimitConstraintModel):
                gcmodel.calc(model, data, gcdata)
                gjac = gcmodel.jacobian(model, data, gcdata)
                gsig = gcmodel.jacobianMatrixProduct(model, data, gcdata, v)
                gtau = gcmodel.jacobianTransposeMatrixProduct(model, data, gcdata, lamb)

                self.assertTrue(np.all(jac == gjac))
                self.assertTrue(np.all(sig == gsig))
                self.assertTrue(np.all(tau == gtau))

    def test_sets_of_constraints(self):
        # Test projection
        # bilat
        cmodel = self.bilat
        set = cmodel.set(cmodel.createData())
        force = np.array([1] * 3)
        p_force = set.project(force)
        self.assertTrue(set.isInside(force))
        self.assertTrue(np.all(p_force == force))

        # fac
        cmodel = self.facm
        set = cmodel.set(cmodel.createData())
        force = np.array([1] * 6)
        p_force = set.project(force)
        p_force = set.project(force)
        self.assertTrue(set.isInside(force))
        self.assertTrue(np.all(p_force == force))

        # jlc
        jld = self.jlc.createData()
        self.jlc.calc(self.model, self.data, jld)
        set = self.jlc.set(jld)
        force_out = np.array([1] * self.jlc.residualSize())
        p_force_out = set.project(force_out)
        self.assertTrue(set.isInside(force_out))
        force_out = np.array([-1] * self.jlc.residualSize())
        p_force_out = set.project(force_out)
        self.assertTrue(not set.isInside(force_out))
        p2_force_out = set.project(p_force_out)
        self.assertTrue(set.isInside(p_force_out))
        self.assertTrue(np.all(p_force_out == p2_force_out))
        force_in = np.array([0] * self.jlc.residualSize())
        p_force_in = set.project(force_in)
        self.assertTrue(set.isInside(p_force_in))
        self.assertTrue(np.all(p_force_in == force_in))

        # jfc
        jfd = self.jfc.createData()
        self.jfc.calc(self.model, self.data, jfd)
        set = self.jfc.set(jfd)

        force_out = np.array([2] * self.jfc.residualSize())
        p_force_out = set.project(force_out)
        self.assertTrue(not set.isInside(force_out))
        p2_force_out = set.project(p_force_out)
        self.assertTrue(set.isInside(p_force_out))
        self.assertTrue(np.all(p2_force_out == p_force_out))
        force_in = np.array([0] * self.jfc.residualSize())
        p_force_in = set.project(force_in)
        self.assertTrue(set.isInside(p_force_in))
        self.assertTrue(np.all(p_force_in == force_in))

        # fp
        self.fp.setFriction(2.0)
        set = self.fp.set(self.fp.createData())
        self.assertTrue(set.mu == 2.0)
        force_out = np.array([0.0, 3.0, 1.0])
        p_force_out = set.project(force_out)
        self.assertTrue(not set.isInside(force_out))
        p2_force_out = set.project(p_force_out)
        self.assertTrue(set.isInside(p_force_out))
        self.assertTrue(np.all(p2_force_out == p_force_out))

        force_in = np.array([0.0, 1.0, 1.0])
        p_force_in = set.project(force_in)
        self.assertTrue(set.isInside(force_in))
        self.assertTrue(np.all(p_force_in == force_in))

    def test_constraints_data(self):
        for cmodel in self.one_of_each:
            cdata = cmodel.createData()
            model_name = cmodel.shortname()
            data_name = cdata.shortname()

            cmodel.calc(self.model, self.data, cdata)

            self.assertTrue(model_name[:-5] == data_name[:-4])
            self.assertTrue(model_name[-5:] == "Model")
            self.assertTrue(data_name[-4:] == "Data")

        for cmodel in [self.bilat, self.facm]:
            cdata = cmodel.createData()
            cmodel.calc(self.model, self.data, cdata)
            self.assertTrue(hasattr(cdata, "constraint_force"))
            self.assertTrue(hasattr(cdata, "oMc1"))
            self.assertTrue(hasattr(cdata, "oMc2"))
            self.assertTrue(hasattr(cdata, "c1Mc2"))
            self.assertTrue(hasattr(cdata, "constraint_position_error"))
            self.assertTrue(hasattr(cdata, "constraint_velocity_error"))
            self.assertTrue(hasattr(cdata, "constraint_acceleration_error"))
            self.assertTrue(hasattr(cdata, "constraint_acceleration_biais_term"))
            self.assertTrue(cdata.oMc1.inverse() * cdata.oMc2 == cdata.c1Mc2)

        cmodel = self.jlc
        cdata = cmodel.createData()
        cmodel.calc(self.model, self.data, cdata)
        self.assertTrue(hasattr(cdata, "constraint_residual"))


if __name__ == "__main__":
    unittest.main()
