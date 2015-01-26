'''
Four inverse-kinematics control law, with a objective of dimension 1 (x
abscissia), dimension 3 (x,y,z coordinates), dimension 6 (SE3 exp integration)
and dimension 6 (R3xSO3 integration, straightline in the cartesian space).
'''

def kineinv_dim1(q0,xdes,N=100):
    '''
    Bring the robot hand to X=xdes, using a gradient descent (which is also a
    Gauss-Newton descent in this trivial case).
    '''
    q = np.copy(q0)
    for i in range(N):
        Mrh = robot.Mrh(q)

        e = Mrh.translation[0,0] - xdes
        J = Mrh.rotation * robot.Jrh(q)[:3,:]
        J = J[0,:]

        qdot = -J.T*e
    
        robot.increment(q,qdot*5e-2)
        robot.display(q)
    print "Residuals = ",robot.Mrh(q).translation[0,0] - xdes

def kineinv_dim3(q0,xdes,N=100):
    '''
    Bring the robot hand to [x,y,z]=xdes using Gauss-Newton descent. Arg q0 is
    the initial robot position (np.matrix Nx1); xdes is the desired position
    (np.matrix 3x1); N is optionnal and is the number of integration steps.
    '''
    q = np.copy(q0)
    for i in range(N):
        Mrh = robot.Mrh(q)

        e = Mrh.translation[0:3,0] - xdes
        J = Mrh.rotation * robot.Jrh(q)[:3,:]

        qdot = -npl.pinv(J)*e
    
        robot.increment(q,qdot*5e-2)
        robot.display(q)
    print "Residuals = ",npl.norm(robot.Mrh(q).translation[:3] - xdes)

def kineinv_dim6(q0,Mdes,straight=True,N=100):
    '''
    Bring the robot hand to the reference placement (position+orientation) Mdes
    using Gauss-Newton descent. Arg q0 is the initial robot position (np.matrix
    Nx1); Mdes is the desired placement (se3.SE3 object); straight is a boolean
    option (true for a straight line in the cartesian space corresponding to
    the exp integration in R3xSO3 -- False for a curved line, corresponding to
    the exp integration in SE3); N is optionnal and is the number of
    integration steps.
    '''
    q = np.copy(q0)
    for i in range(100):
        Mrh = robot.Mrh(q)
        
        if straight:
            ERR = Mdes.inverse()*Mrh
            v = ERR.rotation.T*ERR.translation * -1
            w = ERR.rotation.T*se3.log(ERR.rotation) * -1
            nu = se3.Motion(v,w)
        else:
            nu = se3.log( Mrh.inverse()*Mdes )

        Jrh = robot.Jrh(q)
        qdot = npl.pinv(Jrh)*nu.vector()
        
        robot.increment(q,qdot*5e-2)
        robot.display(q)
    print "Residuals = ",npl.norm(se3.log(robot.Mrh(q).inverse()*Mdes).vector())

# --- MAIN ------------------------------------------------------
import pinocchio as se3
from pinocchio.utils import *

robot = se3.RobotWrapper('../../models/romeo.urdf')
robot.initDisplay()

kineinv_dim1(robot.q0, 2.0)
kineinv_dim3(robot.q0, np.matrix([2.0,0.5,1.0]).T)
kineinv_dim6(robot.q0, se3.SE3(eye(3),np.matrix([2.0,0.5,1.0]).T),straight=False)
kineinv_dim6(robot.q0, se3.SE3(eye(3),np.matrix([2.0,0.5,1.0]).T),straight=True)
