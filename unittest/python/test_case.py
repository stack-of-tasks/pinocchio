import importlib.util
import unittest

import numpy as np
import pinocchio as pin
from pinocchio.utils import isapprox

coal_spec = importlib.util.find_spec("coal")
coal_found = coal_spec is not None
if coal_found:
    import coal

matplotlib_spec = importlib.util.find_spec("matplotlib")
matplotlib_found = matplotlib_spec is not None
if matplotlib_found:
    import matplotlib.pyplot as plt

meshcat_spec = importlib.util.find_spec("meshcat")
meshcat_found = meshcat_spec is not None
if meshcat_found:
    import meshcat
    from pinocchio.visualize import MeshcatVisualizer


def tracefunc(frame, event, arg):
    print(f"{event}, {frame.f_code.co_filename}: {frame.f_lineno}")
    return tracefunc


class PinocchioTestCase(unittest.TestCase):
    def assertApprox(self, a, b, eps=1e-6):
        return self.assertTrue(
            isapprox(a, b, eps),
            f"\n{a}\nis not approximately equal to\n{b}\nwith precision {eps:f}",
        )


class ContactSolverTestCase(PinocchioTestCase):
    def buildStackOfCubesModel(self, masses):
        model = pin.Model()
        n_cubes = len(masses)
        box_dims = np.ones(3)
        for i in range(n_cubes):
            box_mass = masses[i]
            box_inertia = pin.Inertia.FromBox(
                box_mass, box_dims[0], box_dims[1], box_dims[2]
            )
            joint_id = model.addJoint(
                0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "free_flyer_" + str(i)
            )
            model.appendBodyToJoint(joint_id, box_inertia, pin.SE3.Identity())

        friction_coeff = 0.4
        list_cm = []
        for i in range(n_cubes):
            local_trans_box_1 = 0.5 * box_dims
            local_trans_box_2 = 0.5 * box_dims
            local_trans_box_2[2] *= -1
            rot = np.identity(3)
            theta = np.pi / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            for j in range(4):
                local_placement_1 = pin.SE3(np.identity(3), (rot @ local_trans_box_1))
                local_placement_2 = pin.SE3(np.identity(3), (rot @ local_trans_box_2))
                fpcm = pin.PointContactConstraintModel(
                    model, i, local_placement_1, i + 1, local_placement_2
                )
                fpcm.setFriction(friction_coeff)
                list_cm.append(pin.ConstraintModel(fpcm))
                rot = R @ rot

        return model, list_cm

    def setupTest(self, model, constraint_models, q0, v0, tau0, fext, dt):
        # initialize data
        data = model.createData()
        pin.aba(model, data, q0, v0, tau0, fext)
        data.q_in = q0
        data.v_in = v0
        data.tau_in = tau0

        pin.crba(model, data, q0, pin.Convention.WORLD)
        vfree = v0 + dt * pin.aba(model, data, q0, v0, tau0, fext)

        # initialize constraint datas
        constraint_datas = pin.StdVec_ConstraintData()
        for cmodel in constraint_models:
            constraint_datas.append(cmodel.createData())
            cdata = constraint_datas[-1]
            cmodel.calc(model, data, cdata)

        chol = pin.ConstraintCholeskyDecomposition(
            model, data, constraint_models, constraint_datas
        )
        chol.compute(model, data, constraint_models, constraint_datas, 1e-10)
        delassus_matrix = chol.getDelassusOperatorCholeskyExpression().matrix()
        Jc = pin.getConstraintsJacobian(
            model, data, constraint_models, constraint_datas
        )
        g = Jc @ vfree
        idx_cm = 0
        for i, cm in enumerate(constraint_models):
            cd = constraint_datas[i]
            cm_csize = cm.residualSize()
            if cm.shortname() == "PointContactConstraintModel":
                continue
            elif cm.shortname() == "JointFrictionConstraintModel":
                continue
            elif cm.shortname() == "JointLimitConstraintModel":
                g[idx_cm : idx_cm + cm_csize] *= dt
                g[idx_cm : idx_cm + cm_csize] += cd.extract().constraint_residual
            elif cm.shortname() == "PointAnchorConstraintModel":
                g[idx_cm : idx_cm + cm_csize] *= dt
                g[idx_cm : idx_cm + cm_csize] += cd.extract().constraint_position_error
            idx_cm += cm_csize
        return delassus_matrix, g, constraint_datas

    @unittest.skipUnless(coal_found, "Needs Coal.")
    def addFloor(self, geom_model: pin.GeometryModel, visual_model: pin.GeometryModel):
        floor_collision_shape = coal.Halfspace(0, 0, 1, 0)
        M = pin.SE3.Identity()
        floor_collision_object = pin.GeometryObject(
            "floor", 0, 0, M, floor_collision_shape
        )
        floor_collision_object.meshColor = np.array([0.5, 0.5, 0.5, 0.5])
        geom_model.addGeometryObject(floor_collision_object)

        h = 0.01
        floor_visual_shape = coal.Box(20, 20, h)
        Mvis = pin.SE3.Identity()
        Mvis.translation = np.array([0.0, 0.0, -h / 2])
        floor_visual_object = pin.GeometryObject(
            "floor", 0, 0, Mvis, floor_visual_shape
        )
        floor_visual_object.meshColor = np.array([0.5, 0.5, 0.5, 0.4])
        visual_model.addGeometryObject(floor_visual_object)

    @unittest.skipUnless(coal_found, "Needs Coal.")
    def addSystemCollisionPairs(self, model, geom_model, qref):
        """
        Add the right collision pairs of a model, given qref.
        qref is here as a `T-pose`. The function uses this pose to determine
        which objects are in collision in this ref pose.
        If objects are in collision, they are not added as collision pairs,
        as they are considered to always be in collision.
        """
        data = model.createData()
        geom_data = geom_model.createData()
        pin.updateGeometryPlacements(model, data, geom_model, geom_data, qref)
        geom_model.removeAllCollisionPairs()
        num_col_pairs = 0
        for i in range(len(geom_model.geometryObjects)):
            for j in range(i + 1, len(geom_model.geometryObjects)):
                # Don't add collision pair if same object
                if i != j:
                    gobj_i: pin.GeometryObject = geom_model.geometryObjects[i]
                    gobj_j: pin.GeometryObject = geom_model.geometryObjects[j]
                    if gobj_i.name == "floor" or gobj_j.name == "floor":
                        num_col_pairs += 1
                        col_pair = pin.CollisionPair(i, j)
                        geom_model.addCollisionPair(col_pair)
                    else:
                        if gobj_i.parentJoint != gobj_j.parentJoint:
                            # Compute collision between the geometries.
                            # Only add the collision pair if there is no collision.
                            M1 = geom_data.oMg[i]
                            M2 = geom_data.oMg[j]
                            colreq = coal.CollisionRequest()
                            colreq.security_margin = 1e-2  # 1cm of clearance
                            colres = coal.CollisionResult()
                            coal.collide(
                                gobj_i.geometry, M1, gobj_j.geometry, M2, colreq, colres
                            )
                            if not colres.isCollision():
                                num_col_pairs += 1
                                col_pair = pin.CollisionPair(i, j)
                                geom_model.addCollisionPair(col_pair)

    def complete_orthonormal_basis(self, ez, joint_placement):
        ex = joint_placement.rotation[:, 0]
        ey = np.cross(ez, ex)
        if np.linalg.norm(ey) < 1e-6:
            ex = joint_placement.rotation[:, 1]
            ey = np.cross(ez, ex)
        ey /= np.linalg.norm(ey)
        ex = np.cross(ey, ez)
        return ex, ey

    @unittest.skipUnless(coal_found, "Needs Coal.")
    def computeContacts(self, model, geom_model, q):
        data = model.createData()
        geom_data = geom_model.createData()
        pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
        pin.computeCollisions(geom_model, geom_data, False)
        pin.computeContactPatches(geom_model, geom_data)
        contact_constraints = pin.StdVec_ConstraintModel()
        friction_coeff = 0.4
        for i, res in enumerate(geom_data.collisionResults):
            patch_res = geom_data.contactPatchResults[i]
            if res.isCollision() and patch_res.numContactPatches():
                geom_id1, geom_id2 = (
                    geom_model.collisionPairs[i].first,
                    geom_model.collisionPairs[i].second,
                )
                joint_id1 = geom_model.geometryObjects[geom_id1].parentJoint
                joint_id2 = geom_model.geometryObjects[geom_id2].parentJoint
                joint_placement_1 = data.oMi[joint_id1]
                joint_placement_2 = data.oMi[joint_id2]
                patch = patch_res.getContactPatch(0)
                contact_normal = patch.getNormal()
                num_contacts_colpair = min(patch.size(), 4)
                for contact_id in range(num_contacts_colpair):
                    contact_position = patch.getPoint(contact_id)
                    ex_i, ey_i = self.complete_orthonormal_basis(
                        contact_normal, joint_placement_1
                    )
                    ex_i = np.expand_dims(ex_i, axis=1)
                    ey_i = np.expand_dims(ey_i, axis=1)
                    normal_i = np.expand_dims(contact_position, axis=1)
                    R_i = np.concatenate((ex_i, ey_i, normal_i), axis=1)
                    R_i1 = np.dot(joint_placement_1.rotation.T, R_i)
                    R_i2 = np.dot(joint_placement_2.rotation.T, R_i)
                    pos_i1 = joint_placement_1.rotation.T @ (
                        contact_position - joint_placement_1.translation
                    )
                    pos_i2 = joint_placement_2.rotation.T @ (
                        contact_position - joint_placement_2.translation
                    )
                    placement_i1 = pin.SE3(R_i1, pos_i1)
                    placement_i2 = pin.SE3(R_i2, pos_i2)
                    contact_model_i = pin.PointContactConstraintModel(
                        model, joint_id2, placement_i2, joint_id1, placement_i1
                    )
                    contact_model_i.set = pin.CoulombFrictionCone(friction_coeff)
                    contact_constraints.append(contact_model_i)
        return contact_constraints

    @unittest.skipUnless(meshcat_found, "Needs meshcat.")
    def createVisualizer(
        self,
        model: pin.GeometryModel,
        geom_model: pin.GeometryModel,
        visual_model: pin.GeometryModel,
    ):
        viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
        viewer.delete()
        for obj in visual_model.geometryObjects:
            color = np.random.rand(4)
            color[3] = 1.0
            obj.meshColor = color
        vizer: MeshcatVisualizer = MeshcatVisualizer(model, geom_model, visual_model)
        vizer.initViewer(viewer=viewer, open=False, loadModel=True)
        return vizer, viewer

    @unittest.skipUnless(matplotlib_found, "Needs Matplotlib.")
    def plotContactSolver(self, solver):
        stats: pin.SolverStats = solver.stats
        if stats.size() > 0:
            plt.figure()
            it = solver.solution.iterations
            primal_feas = solver.solution.primal_feasibility
            dual_feas = solver.solution.dual_feasibility
            plt.cla()
            plt.title(
                f"it = {it}, abs res = {primal_feas:.2e}, rel res = {dual_feas:.2e}"
            )
            plt.plot(stats.complementarity, label="complementarity")
            plt.plot(stats.primal_feasibility, label="primal feas")
            plt.plot(stats.dual_feasibility, label="dual feas")
            plt.yscale("log")
            plt.legend()
            plt.ion()
            plt.show()
