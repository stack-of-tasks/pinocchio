'''
Create a simulation environment for a N-pendulum.
Example of use:

env = Pendulum(N)
env.reset()

for i in range(1000):
   env.step(zero(env.nu))
   env.render()

'''


from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as pin
import gepetto.corbaserver
from display import Display
from numpy.linalg import pinv,norm,inv
import time


class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint 
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Pendulum:
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3). 
    The configuration is nq=7. The velocity is the same. 
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).
    
    See tp1.py for an example of use.
    '''

    def __init__(self,nbJoint=1):
        '''Create a Pinocchio model of a N-pendulum, with N the argument <nbJoint>.'''
        self.viewer     = Display()
        self.visuals    = []
        self.model      = pin.Model()
        self.createPendulum(nbJoint)
        self.data       = self.model.createData()

        self.q0         = zero(self.model.nq)

        self.DT         = 5e-2   # Step length
        self.NDT        = 2      # Number of Euler steps per integration (internal)
        self.Kf         = .10    # Friction coefficient
        self.vmax       = 8.0    # Max velocity (clipped if larger)
        self.umax       = 2.0    # Max torque   (clipped if larger)
        self.withSinCos = False   # If true, state is [cos(q),sin(q),qdot], else [q,qdot]

    def createPendulum(self,nbJoint,rootId=0,prefix='',jointPlacement=None):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId
        jointPlacement     = jointPlacement if jointPlacement!=None else pin.SE3.Identity()
        length = 1.0
        mass = length
        inertia = pin.Inertia(mass,
                              np.matrix([0.0,0.0,length/2]).T,
                              mass/5*np.diagflat([ 1e-2,length**2,  1e-2 ]) )

        for i in range(nbJoint):
            istr = str(i)
            name               = prefix+"joint"+istr
            jointName,bodyName = [name+"_joint",name+"_body"]
            jointId = self.model.addJoint(jointId,pin.JointModelRY(),jointPlacement,jointName)
            self.model.appendBodyToJoint(jointId,inertia,pin.SE3.Identity())
            try:self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere'+istr, 0.15,colorred)
            except: pass
            self.visuals.append( Visual('world/'+prefix+'sphere'+istr,jointId,pin.SE3.Identity()) )
            try:self.viewer.viewer.gui.addCapsule('world/'+prefix+'arm'+istr, .1,.8*length,color)
            except:pass
            self.visuals.append( Visual('world/'+prefix+'arm'+istr,jointId,
                                        pin.SE3(eye(3),np.matrix([0.,0.,length/2]))))
            jointPlacement     = pin.SE3(eye(3),np.matrix([0.0,0.0,length]).T)

        self.model.addFrame( pin.Frame('tip',jointId,0,jointPlacement,pin.FrameType.OP_FRAME) )

    def display(self,q):
        pin.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()


    @property
    def nq(self): return self.model.nq
    @property
    def nv(self): return self.model.nv
    @property
    def nx(self): return self.nq+self.nv
    @property
    def nobs(self): return self.nx+self.withSinCos
    @property
    def nu(self): return self.nv

    def reset(self,x0=None):
        if x0 is None: 
            q0 = np.pi*(rand(self.nq)*2-1)
            v0 = rand(self.nv)*2-1
            x0 = np.vstack([q0,v0])
        assert len(x0)==self.nx
        self.x = x0.copy()
        self.r = 0.0
        return self.obs(self.x)

    def step(self,u):
        assert(len(u)==self.nu)
        _,self.r = self.dynamics(self.x,u)
        return self.obs(self.x),self.r

    def obs(self,x):
        if self.withSinCos:
            return np.vstack([ np.vstack([np.cos(qi),np.sin(qi)]) for qi in x[:self.nq] ] 
                             + [x[self.nq:]],)
        else: return x.copy()

    def tip(self,q):
        '''Return the altitude of pendulum tip'''
        pin.framesKinematics(self.model,self.data,q)
        return self.data.oMf[1].translation[2,0]

    def dynamics(self,x,u,display=False):
        '''
        Dynamic function: x,u -> xnext=f(x,y).
        Put the result in x (the initial value is destroyed). 
        Also compute the cost of making this step.
        Return x for convenience along with the cost.
        '''

        modulePi = lambda th: (th+np.pi)%(2*np.pi)-np.pi
        sumsq    = lambda x : np.sum(np.square(x))

        cost = 0.0
        q = modulePi(x[:self.nq])
        v = x[self.nq:]
        u = np.clip(np.reshape(np.matrix(u),[self.nu,1]),-self.umax,self.umax)


        DT = self.DT/self.NDT
        for i in range(self.NDT):
            pin.computeAllTerms(self.model,self.data,q,v)
            M   = self.data.M
            b   = self.data.nle
            #tau = u-self.Kf*v
            a   = inv(M)*(u-self.Kf*v-b)

            v    += a*DT
            q    += v*DT
            cost += (sumsq(q) + 1e-1*sumsq(v) + 1e-3*sumsq(u))*DT

            if display:
                self.display(q)
                time.sleep(1e-4)

        x[:self.nq] = modulePi(q)
        x[self.nq:] = np.clip(v,-self.vmax,self.vmax)
        
        return x,-cost
     
    def render(self):
        q = self.x[:self.nq]
        self.display(q)
        time.sleep(self.DT/10)

        
