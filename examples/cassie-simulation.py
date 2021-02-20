import numpy as np
import hppfcl as fcl
import pinocchio
from pinocchio import buildModelFromSdf, buildGeomFromSdf, neutral, JointModelFreeFlyer
from pinocchio.visualize import GepettoVisualizer
from pinocchio import GeometryType
from time import sleep

from os.path import join, dirname, abspath

pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
sdf_filename = pinocchio_model_dir + "/cassie-gazebo-sim/cassie/cassie_v2.sdf"        
package_dir = pinocchio_model_dir + "/cassie-gazebo-sim"


(model,constraint_models) = buildModelFromSdf(sdf_filename, JointModelFreeFlyer())
print "model parsed"

q0 = neutral(model)
q0[2] = 1.104
q0[9] = 0.298
q0[20] = 0.298
q0[17] = -1.398
q0[28] = -1.398
q0[26] = -1.398; q0[27] = -q0[26]
q0[15] = -1.398; q0[16] = -q0[15]

model.q0 = q0.copy()

data = pinocchio.Data(model)

pinocchio.forwardKinematics(model, data, q0)

foot_joints = ["left-foot-op", "right-foot-op"]
foot_joint_ids = [model.getJointId(joint_name) for joint_name in foot_joints]
R = data.oMi[model.getJointId("right-foot-op")].rotation

z_value = -data.oMi[model.getJointId("right-foot-op")].translation[2]
front_x = 0.1
back_x = -0.06

front_placement = pinocchio.SE3(np.identity(3),
                                np.dot(np.linalg.inv(R), np.array([front_x, 0., z_value])))

back_placement = pinocchio.SE3(np.identity(3),
                               np.dot(np.linalg.inv(R), np.array([back_x, 0., z_value])))


#Add contact model for contact with ground

for joint_id in foot_joint_ids:
    contact_model_lf1 = pinocchio.RigidContactModel(pinocchio.ContactType.CONTACT_3D,
                                                    joint_id,
                                                    front_placement,
                                                    0,
                                                    data.oMi[joint_id] * front_placement,
                                                    pinocchio.ReferenceFrame.LOCAL)
    
    contact_model_lf2 = pinocchio.RigidContactModel(pinocchio.ContactType.CONTACT_3D,
                                                    joint_id,
                                                    back_placement,
                                                    0,
                                                    data.oMi[joint_id] * back_placement,
                                                    pinocchio.ReferenceFrame.LOCAL)
    
    constraint_models.extend([contact_model_lf1, contact_model_lf2])


contact_model_pelvis = pinocchio.RigidContactModel(pinocchio.ContactType.CONTACT_3D,
                                                   1,
                                                   pinocchio.SE3.Identity(),
                                                   0,
                                                   pinocchio.SE3(np.identity(3),
                                                                 np.array([0, 0., 1.07143])),
                                                   pinocchio.ReferenceFrame.LOCAL)
    
visual_model = buildGeomFromSdf(model, constraint_models, sdf_filename,
                                GeometryType.VISUAL, package_dir)

collision_model = visual_model
viz = GepettoVisualizer(model, collision_model, visual_model)

viz.initViewer()
viz.loadViewerModel("pinocchio")
gui = viz.viewer.gui
viz.display(model.q0)
window_id = viz.viewer.gui.getWindowID('python-pinocchio')

viz.viewer.gui.setBackgroundColor1(window_id, [1., 1., 1., 1.])
viz.viewer.gui.setBackgroundColor2(window_id, [1., 1., 1., 1.])
viz.viewer.gui.addFloor('hpp-gui/floor')

viz.viewer.gui.setScale('hpp-gui/floor', [0.5, 0.5, 0.5])
viz.viewer.gui.setColor('hpp-gui/floor', [0.7, 0.7, 0.7, 1.])
viz.viewer.gui.setLightingMode('hpp-gui/floor', 'OFF')



viz.display(q0)







constraint_datas = [cm.createData() for cm in constraint_models]

def check_joint(model,n, ncycles=1):
    for i in xrange(200*ncycles):
        q1=q0.copy();
        q1[ model.idx_qs[n] ] = i*3.14/100.;
        viz.display(q1)
        sleep(0.005)

q = q0.copy()

pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
kkt_constraint = pinocchio.ContactCholeskyDecomposition(model,constraint_models)
constraint_dim = sum([cm.size() for cm in constraint_models])
N=100000
eps = 1e-10
mu = 1e-4
#q_sol = (q[:] + np.pi) % np.pi - np.pi
q_sol = q.copy()
viz.display(q_sol)

#Bring CoM between the two feet.

def squashing(model, data, q_in):
    q = q_in.copy()
    y = np.ones((constraint_dim))

    N_full = 200
    
    #Decrease CoMz by 0.2
    com_drop_amp = 0.3
    pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
    com_base = data.com[0].copy()
    
    com_des = lambda k: com_base - np.array([0., 0.,
                                             np.abs(com_drop_amp*np.sin(2.*np.pi*k/(N_full)))])
    for k in range(N):
        pinocchio.computeAllTerms(model,data,q,np.zeros(model.nv))
        pinocchio.computeJointJacobians(model,data,q)
        com_act = data.com[0].copy()
        com_err = com_act - com_des(k)
        kkt_constraint.compute(model, data, constraint_models, constraint_datas, mu)
        constraint_value = np.concatenate([cd.c1Mc2.translation for cd in constraint_datas])
        J = np.vstack([pinocchio.getFrameJacobian(model,data,cm.joint1_id,
                                                  cm.joint1_placement,
                                                  cm.reference_frame)[:3,:] for cm in constraint_models])
        primal_feas = np.linalg.norm(constraint_value,np.inf)
        dual_feas = np.linalg.norm(J.T.dot(constraint_value + y),np.inf)
        if primal_feas < eps and dual_feas < eps:
            print("Convergence achieved")
            break
        print("constraint_value:",np.linalg.norm(constraint_value))
        print ("com_error:", np.linalg.norm(com_err))
        rhs = np.concatenate([-constraint_value - y*mu, 10.*com_err, np.zeros(model.nv-3)])
        dz = kkt_constraint.solve(rhs) 
        dy = dz[:constraint_dim]
        dq = dz[constraint_dim:]
        alpha = 1.
        q = pinocchio.integrate(model,q,-alpha*dq)
        y -= alpha*(-dy + y)
        viz.display(q)
        sleep(0.05)

squashing(model, data, model.q0)
