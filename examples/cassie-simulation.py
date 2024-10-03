from time import sleep

import numpy as np
import pinocchio
import example_robot_data
from pinocchio.visualize import GepettoVisualizer

robot = example_robot_data.load("cassie")

constraint_models = robot.constraint_models

data = pinocchio.Data(robot.model)

pinocchio.forwardKinematics(robot.model, robot.data, robot.q0)

foot_joints = ["left-plantar-foot-joint", "right-plantar-foot-joint"]
foot_joint_ids = [robot.model.getJointId(joint_name) for joint_name in foot_joints]

front_placement = pinocchio.SE3(np.identity(3), np.array([-0.1, 0.11, 0.0]))

back_placement = pinocchio.SE3(np.identity(3), np.array([0.03, -0.0, 0.0]))

foot_length = np.linalg.norm(front_placement.translation - back_placement.translation)


# Add contact robot.model for contact with ground

mid_point = np.zeros(3)

for joint_id in foot_joint_ids:
    joint2_placement = robot.data.oMi[joint_id] * front_placement
    joint2_placement.translation[2] = 0
    contact_model_lf1 = pinocchio.RigidConstraintModel(
        pinocchio.ContactType.CONTACT_3D,
        robot.model,
        joint_id,
        front_placement,
        0,
        joint2_placement,
        pinocchio.ReferenceFrame.LOCAL,
    )

    mid_point += joint2_placement.translation
    joint2_placement.translation[0] -= foot_length

    contact_model_lf2 = pinocchio.RigidConstraintModel(
        pinocchio.ContactType.CONTACT_3D,
        robot.model,
        joint_id,
        back_placement,
        0,
        joint2_placement,
        pinocchio.ReferenceFrame.LOCAL,
    )

    mid_point += joint2_placement.translation
    constraint_models.extend([contact_model_lf1, contact_model_lf2])

mid_point /= 4.0


robot.setVisualizer(GepettoVisualizer())
robot.initViewer()
robot.loadViewerModel("pinocchio")
gui = robot.viewer.gui
robot.display(robot.q0)
q0 = robot.q0.copy()
constraint_datas = pinocchio.StdVec_RigidConstraintData()
for cm in constraint_models:
    constraint_datas.append(cm.createData())


def update_axis(q):
    pinocchio.forwardKinematics(robot.model, robot.data, q)
    for j, cm in enumerate(robot.constraint_models):
        pos1 = robot.data.oMi[cm.joint1_id] * cm.joint1_placement
        pos2 = robot.data.oMi[cm.joint2_id] * cm.joint2_placement
        name1 = "hpp-gui/cm1_" + str(j)
        name2 = "hpp-gui/cm2_" + str(j)
        gui.applyConfiguration(name1, list(pinocchio.SE3ToXYZQUAT(pos1)))
        gui.applyConfiguration(name2, list(pinocchio.SE3ToXYZQUAT(pos2)))
        gui.refresh()


def check_joint(model, nj):
    for k in range(model.joints[nj].nv):
        for res in range(200):
            q1 = robot.q0.copy()
            theta = res * 2.0 * np.pi / 200.0
            v = np.zeros(robot.model.nv)
            v[robot.model.idx_vs[nj] + k] = theta
            q1 = pinocchio.integrate(robot.model, q1, v)
            robot.display(q1)
            update_axis(q1)
            sleep(0.005)


q = q0.copy()

pinocchio.computeAllTerms(robot.model, robot.data, q, np.zeros(robot.model.nv))
kkt_constraint = pinocchio.ContactCholeskyDecomposition(robot.model, constraint_models)
constraint_dim = sum([cm.size() for cm in constraint_models])
N = 1000
eps = 1e-6
mu = 1e-4
# q_sol = (q[:] + np.pi) % np.pi - np.pi
q_sol = q.copy()

window_id = robot.viewer.gui.getWindowID("python-pinocchio")
robot.viewer.gui.setBackgroundColor1(window_id, [1.0, 1.0, 1.0, 1.0])
robot.viewer.gui.setBackgroundColor2(window_id, [1.0, 1.0, 1.0, 1.0])
robot.viewer.gui.addFloor("hpp-gui/floor")
robot.viewer.gui.setScale("hpp-gui/floor", [0.5, 0.5, 0.5])
robot.viewer.gui.setColor("hpp-gui/floor", [0.7, 0.7, 0.7, 1.0])
robot.viewer.gui.setLightingMode("hpp-gui/floor", "OFF")


axis_size = 0.08
radius = 0.005
transparency = 0.5

for j, cm in enumerate(constraint_models):
    pos1 = robot.data.oMi[cm.joint1_id] * cm.joint1_placement
    pos2 = robot.data.oMi[cm.joint2_id] * cm.joint2_placement
    name1 = "hpp-gui/cm1_" + str(j)
    name2 = "hpp-gui/cm2_" + str(j)
    red_color = 1.0 * float(j) / float(len(constraint_models))
    print(red_color)
    robot.viewer.gui.addXYZaxis(
        name1, [red_color, 1.0, 1.0 - red_color, transparency], radius, axis_size
    )
    robot.viewer.gui.addXYZaxis(
        name2, [red_color, 1.0, 1.0 - red_color, transparency], radius, axis_size
    )

    gui.applyConfiguration(name1, list(pinocchio.SE3ToXYZQUAT(pos1)))
    gui.applyConfiguration(name2, list(pinocchio.SE3ToXYZQUAT(pos2)))
    gui.refresh()
    gui.setVisibility(name1, "OFF")
    gui.setVisibility(name2, "OFF")


mid_point_name = "hpp-gui/mid_point"
mid_point_pos = pinocchio.SE3(np.identity(3), mid_point)
robot.viewer.gui.addXYZaxis(
    mid_point_name, [0.0, 0.0, 1.0, transparency], radius, axis_size
)


gui.applyConfiguration(mid_point_name, list(pinocchio.SE3ToXYZQUAT(mid_point_pos)))
gui.setVisibility(mid_point_name, "ALWAYS_ON_TOP")

robot.display(q_sol)

# Bring CoM between the two feet.

mass = robot.data.mass[0]

com_base = robot.data.com[0].copy()
com_base[:2] = mid_point[:2]
com_drop_amp = 0.1


def com_des(k):
    return com_base - np.array([0.0, 0.0, np.abs(com_drop_amp)])


def squashing(model, data, q_in, Nin=N, epsin=eps, verbose=True):
    q = q_in.copy()
    y = np.ones(constraint_dim)

    _N_full = 200

    # Decrease CoMz by 0.2
    pinocchio.computeAllTerms(robot.model, robot.data, q, np.zeros(robot.model.nv))
    kp = np.array([1.0, 1.0, 0.1])

    for k in range(Nin):
        pinocchio.computeAllTerms(robot.model, robot.data, q, np.zeros(robot.model.nv))
        pinocchio.computeJointJacobians(robot.model, robot.data, q)
        com_act = robot.data.com[0].copy()
        com_err = com_act - com_des(k)
        kkt_constraint.compute(
            robot.model, robot.data, constraint_models, constraint_datas, mu
        )
        constraint_value1 = np.concatenate(
            [pinocchio.log(cd.c1Mc2) for cd in constraint_datas[:-4]]
        )
        constraint_value2 = np.concatenate(
            [cd.c1Mc2.translation for cd in constraint_datas[-4:]]
        )
        constraint_value = np.concatenate([constraint_value1, constraint_value2])

        J1 = np.vstack(
            [
                pinocchio.getFrameJacobian(
                    robot.model,
                    data,
                    cm.joint1_id,
                    cm.joint1_placement,
                    cm.reference_frame,
                )
                for cm in constraint_models[:-4]
            ]
        )
        J2 = np.vstack(
            [
                pinocchio.getFrameJacobian(
                    robot.model,
                    data,
                    cm.joint1_id,
                    cm.joint1_placement,
                    cm.reference_frame,
                )[:3, :]
                for cm in constraint_models[-4:]
            ]
        )
        J = np.vstack([J1, J2])
        primal_feas = np.linalg.norm(constraint_value, np.inf)

        dual_feas = np.linalg.norm(J.T.dot(constraint_value + y), np.inf)
        if primal_feas < epsin and dual_feas < epsin:
            print("Convergence achieved")
            break
        if verbose:
            print("constraint_value:", np.linalg.norm(constraint_value))
            print("com_error:", np.linalg.norm(com_err))
            print("com_des", com_des(k))
            print("com_act", com_act)
        rhs = np.concatenate(
            [-constraint_value - y * mu, kp * com_err, np.zeros(robot.model.nv - 3)]
        )
        dz = kkt_constraint.solve(rhs)
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]
        alpha = 1.0
        q = pinocchio.integrate(robot.model, q, -alpha * dq)
        y -= alpha * (-dy + y)
        robot.display(q)
        update_axis(q)
        gui.applyConfiguration(
            mid_point_name,
            list(pinocchio.SE3ToXYZQUAT(pinocchio.SE3(np.identity(3), com_des(k)))),
        )
        sleep(0.0)
    return q


qin = robot.q0.copy()
for k in range(1000):
    qout = squashing(robot.model, data, qin)
    qout[3:6] = 0.0
    qout[6] = 1.0
    qin = qout.copy()

qout = squashing(robot.model, data, qin, Nin=100000, epsin=1e-10, verbose=False)
