import importlib.util
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from test_case import ContactSolverTestCase as TestCase

matplotlib_spec = importlib.util.find_spec("matplotlib")
matplotlib_found = matplotlib_spec is not None

meshcat_spec = importlib.util.find_spec("meshcat")
meshcat_found = meshcat_spec is not None


class TestADMM(TestCase):
    def test_box(self):
        model, constraint_models = self.buildStackOfCubesModel([1e-3])
        q0 = pin.neutral(model)
        v0 = np.zeros(model.nv)
        tau0 = np.zeros(model.nv)
        fext = [pin.Force.Zero() for _ in range(model.njoints)]
        dt = 1e-3

        delassus_matrix, g, constraint_datas = self.setupTest(
            model, constraint_models, q0, v0, tau0, fext, dt
        )
        delassus = pin.DelassusOperatorDense(delassus_matrix)
        dim_pb = g.shape[0]
        solver = pin.ADMMConstraintSolver(dim_pb)
        settings = pin.ADMMSolverSettings()
        settings.absolute_feasibility_tol = 1e-13
        settings.relative_feasibility_tol = 1e-14
        settings.absolute_complementarity_tol = 1e-13
        settings.relative_complementarity_tol = 1e-14
        settings.lanczos_size = g.size
        result = pin.ADMMSolverResult()
        solver.solve(delassus, g, constraint_models, constraint_datas, settings, result)

    @unittest.skipUnless(pin.WITH_COLLISION, "Needs collision support")
    def test_cassie(self, display=False, stat_record=True):
        current_dir = Path(__file__).parent
        model_dir = current_dir / "../models/"
        model_path = model_dir / "closed_chain.xml"
        constraint_models = pin.StdVec_ConstraintModel()

        # Parsing model, constraint models and geometry model from xml description
        model, constraint_models_dict, geom_model, visual_model = (
            pin.buildModelsAndConstraintsFromMJCF(model_path)
        )

        # Adding all constraintds would be
        for typed_constraint_models in constraint_models_dict.values():
            for cm in typed_constraint_models:
                constraint_models.append(pin.ConstraintModel(cm))
        # Adding only point anchor constraints to the list of constraints
        # for bpcm in constraint_models_dict['point_anchor_constraint_models']:
        #     constraint_models.append(pin.ConstraintModel(bpcm))

        q0 = model.referenceConfigurations["home"]
        v0 = np.zeros(model.nv)
        tau0 = np.zeros(model.nv)
        fext = [pin.Force.Zero() for _ in range(model.njoints)]
        self.addFloor(geom_model, visual_model)
        self.addSystemCollisionPairs(model, geom_model, q0)
        dt = 1e-3

        # adding joint limit constraints
        active_joints_limits = [i for i in range(1, model.njoints)]
        jlcm = pin.JointLimitConstraintModel(model, active_joints_limits)
        jlcm.makeSelectionFilteredByLimitProximity(q0)
        constraint_models.append(pin.ConstraintModel(jlcm))

        # adding friction on joints
        active_joints_friction = [i for i in range(1, model.njoints)]
        fjcm = pin.JointFrictionConstraintModel(model, active_joints_friction)
        constraint_models.append(pin.ConstraintModel(fjcm))

        # Adding constraints from points contacts
        contact_constraints = self.computeContacts(model, geom_model, q0)
        for fpcm in contact_constraints:
            constraint_models.append(pin.ConstraintModel(fpcm))

        delassus_matrix, g, constraint_datas = self.setupTest(
            model, constraint_models, q0, v0, tau0, fext, dt
        )
        delassus = pin.DelassusOperatorDense(delassus_matrix)

        csize = 0
        for cm in constraint_models:
            csize += cm.residualSize()
        self.assertTrue(
            delassus.matrix().shape[0] == csize,
            "constraint problem is of wrong size.",
        )

        dim_pb = g.shape[0]
        self.assertTrue(dim_pb == csize, "constraint problem is of wrong size")
        solver = pin.ADMMConstraintSolver(dim_pb)
        settings = pin.ADMMSolverSettings()
        settings.absolute_feasibility_tol = 1e-8
        settings.relative_feasibility_tol = 1e-12
        settings.absolute_complementarity_tol = 1e-10
        settings.relative_complementarity_tol = 1e-12
        settings.rho_momentum = 0.9
        settings.anderson_capacity = 4
        settings.lanczos_size = g.size
        result = pin.ADMMSolverResult()

        has_converged = solver.solve(
            delassus, g, constraint_models, constraint_datas, settings, result
        )

        primal_sol = result.retrieveConstraintImpulses()
        print(f"{primal_sol=}")

        print(f"{result.iterations=}")
        print(f"{result.primal_feasibility=}")
        print(f"{result.dual_feasibility=}")
        print(f"{result.complementarity=}")
        self.assertTrue(has_converged, "Solver did not converge.")

        if stat_record and matplotlib_found:
            self.plotContactSolver(solver)

        if display and meshcat_found:
            vizer, _ = self.createVisualizer(model, geom_model, geom_model)
            vizer.display(q0)


if __name__ == "__main__":
    unittest.main()
