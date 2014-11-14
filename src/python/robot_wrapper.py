import numpy as np
import libpinocchio_pywrap as se3
import utils
from explog import exp

class RobotWrapper:
    def __init__(self,filename):
        self.model = se3.buildModelFromUrdf(filename,True)
        self.data = self.model.createData()
        self.v0 = utils.zero(self.nv)
        self.q0 = np.matrix( [
            0, 0, 0.840252, 0, 0, 0, 1,                        # Free flyer
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0,   # left leg   
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0, 0,   # right leg  
            0,                                               # chest
            1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2,         # left arm   
            0, 0, 0, 0,                                      # head
            1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2,          # right arm  
            ] ).T
        self.opCorrespondances = { "lh": "LWristPitch",
                                   "rh": "RWristPitch",
                                   "rf": "RAnkleRoll",
                                   "lf": "LAnkleRoll",
                                   }
        for op,name in self.opCorrespondances.items():
            self.__dict__[op] = self.index(name)

    def increment(self,q,dq):
        M = se3.SE3( se3.Quaternion(q[6,0],q[3,0],q[4,0],q[5,0]).matrix(), q[:3])
        dM = exp(dq[:6])
        M = M*dM
        q[:3] = M.translation
        q[3:7] = se3.Quaternion(M.rotation).coeffs()
        q[7:] += dq[6:]

    def index(self,name):
        return [ i for i,n in enumerate(self.model.names) if n == name ][0]
                   
    @property
    def nq(self): 
        return self.model.nq
    @property
    def nv(self): 
        return self.model.nv

    def com(self,q):
        return se3.centerOfMass(self.model,self.data,q)
    def Jcom(self,q):
        return se3.centerOfMass(self.model,self.data,q)

    def mass(self,q):
        return se3.crba(self.model,self.data,q)
    def biais(self,q,v):
        return se3.rnea(self.model,self.data,q,v,self.v0)
    def gravity(self,q):
        return se3.rnea(self.model,self.data,q,self.v0,self.v0)

    def position(self,q,index):
        se3.kinematics(self.model,self.data,q,self.v0)
        return self.data.oMi[index]
    def velocity(self,q,v,index):
        se3.kinematics(self.model,self.data,q,v)
        return self.data.v[index]
    def jacobian(self,q,index):
        return se3.jacobian(self.model,self.data,index,q,True)


    # --- SHORTCUTS ---
    def Mrh(self,q):
        return self.position(q,self.rh)
    def Jrh(self,q):
        return self.jacobian(q,self.rh)
    def wJrh(self,q):
        return se3.jacobian(self.model,self.data,self.rh,q,False)
    def vrh(self,q,v):
        return self.velocity(q,v,self.rh)

    def Jlh(self,q):
        return self.jacobian(q,self.lh)
    def Mlh(self,q):
        return self.position(q,self.lh)

    def Jlf(self,q):
        return self.jacobian(q,self.lf)
    def Mlf(self,q):
        return self.position(q,self.lf)
    def Jrf(self,q):
        return self.jacobian(q,self.rf)
    def Mrf(self,q):
        return self.position(q,self.rf)


    # --- VIEWER ---
    def initDisplay(self):
        import robotviewer
        try:
            self.viewer=robotviewer.client('XML-RPC')
            self.viewer.updateElementConfig('TEST',[0,]*6)
        except:
            if 'viewer' in self.__dict__: del self.viewer
            print "Error while starting the viewer client. "
            print "Check wheter RobotViewer is properly started (as XML-RPC server)"

    def display(self,q):
        if 'viewer' not in self.__dict__: return
        # Update the robot geometry.
        se3.kinematics(self.model,self.data,q,self.v0)        
        # Iteratively place the robot bodies.
        for i in range(1,self.model.nbody):
            xyz = self.data.oMi[i].translation
            rpy = utils.matrixToRpy(self.data.oMi[i].rotation)
            self.viewer.updateElementConfig('Romeo'+self.model.names[i],
                                            [float(xyz[0,0]), float(xyz[1,0]), float(xyz[2,0]), 
                                             float(rpy[0,0]), float(rpy[1,0]), float(rpy[2,0]) ])


__all__ = [ 'RobotWrapper' ]
