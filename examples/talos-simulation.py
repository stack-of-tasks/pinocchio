import numpy as np
import hppfcl as fcl
import pinocchio
from example_robot_data import loadTalos
from pinocchio.visualize import GepettoVisualizer
from pinocchio import GeometryType
from time import sleep

from os.path import join, dirname, abspath

robot = loadTalos()
model = robot.model
data = robot.data

robot.q0 = robot.model.referenceConfigurations["half_sitting"]

pinocchio.forwardKinematics(model, data, robot.q0)

foot_joints = ["leg_right_6_joint", "leg_left_6_joint"]
foot_joint_ids = [model.getJointId(joint_name) for joint_name in foot_joints]
pinocchio.forwardKinematics(model, data, robot.model.referenceConfigurations["half_sitting"])

constraint_models = []

#Add contact model for contact with ground


for joint_id in foot_joint_ids:
    R = data.oMi[joint_id].rotation
    tr = -data.oMi[joint_id].translation
    foot_placement = pinocchio.SE3(np.linalg.inv(R),
                                   np.dot(np.linalg.inv(R), tr))
    contact_model_lf1 = pinocchio.RigidContactModel(pinocchio.ContactType.CONTACT_6D,
                                                    joint_id,
                                                    foot_placement)

    constraint_models.extend([contact_model_lf1])

robot.initViewer()
robot.loadViewerModel("pinocchio")
gui = robot.viewer.gui
robot.display(robot.q0)
window_id = robot.viewer.gui.getWindowID('python-pinocchio')

robot.viewer.gui.setBackgroundColor1(window_id, [1., 1., 1., 1.])
robot.viewer.gui.setBackgroundColor2(window_id, [1., 1., 1., 1.])
robot.viewer.gui.addFloor('hpp-gui/floor')

robot.viewer.gui.setScale('hpp-gui/floor', [0.5, 0.5, 0.5])
robot.viewer.gui.setColor('hpp-gui/floor', [0.7, 0.7, 0.7, 1.])
robot.viewer.gui.setLightingMode('hpp-gui/floor', 'OFF')

robot.display(robot.q0)

constraint_datas = [cm.createData() for cm in constraint_models]

q = robot.q0.copy()

pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
kkt_constraint = pinocchio.ContactCholeskyDecomposition(model,constraint_models)
constraint_dim = sum([cm.size() for cm in constraint_models])
N=100000
eps = 1e-10
mu = 0.
#q_sol = (q[:] + np.pi) % np.pi - np.pi
q_sol = q.copy()
robot.display(q_sol)

#Bring CoM between the two feet.

mass = data.mass[0]

def squashing(model, data, q_in):
    q = q_in.copy()
    y = np.ones((constraint_dim))

    N_full = 200
    
    #Decrease CoMz by 0.2
    com_drop_amp = 0.1
    pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
    com_base = data.com[0].copy()
    kp = 1.
    speed = 1.
    com_des = lambda k: com_base - np.array([0., 0.,
                                             np.abs(com_drop_amp*np.sin(2.*np.pi*k*speed/(N_full)))])
    for k in range(N):
        pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
        pinocchio.computeJointJacobians(model,data,q)
        pinocchio.computeJointJacobians(model,data,q)
        com_act = data.com[0].copy()
        com_err = com_act - com_des(k)
        kkt_constraint.compute(model, data, constraint_models, constraint_datas, mu)
        constraint_value = np.concatenate([pinocchio.log6(cd.c1Mc2) for cd in constraint_datas])
        J = np.vstack([pinocchio.getFrameJacobian(model,data,cm.joint1_id,
                                                  cm.joint1_placement,
                                                  cm.reference_frame) for cm in constraint_models])
        primal_feas = np.linalg.norm(constraint_value,np.inf)
        dual_feas = np.linalg.norm(J.T.dot(constraint_value + y),np.inf)
        print ("primal_feas:",primal_feas)
        print ("dual_feas:",dual_feas)
        #if primal_feas < eps and dual_feas < eps:
        #    print("Convergence achieved")
        #    break
        print("constraint_value:",np.linalg.norm(constraint_value))
        print ("com_error:", np.linalg.norm(com_err))
        rhs = np.concatenate([-constraint_value - y*mu, kp*mass*com_err,
                              np.zeros(model.nv-3)])
        dz = kkt_constraint.solve(rhs)
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]
        alpha = 1.
        q = pinocchio.integrate(model,q,-alpha*dq)
        y -= alpha*(-dy + y)
        robot.display(q)
        sleep(0.05)
    return q

q_new = squashing(model, data, robot.q0)
