import numpy as np
import libpinocchio_pywrap as se3
import utils
from explog import exp

class RobotWrapper:
    def __init__(self,filename):
        self.modelFileName = filename
        self.model = se3.buildModelFromUrdf(filename,True)
        self.data = self.model.createData()
        self.v0 = utils.zero(self.nv)
        self.q0 = utils.zero(self.nq)

    def increment(self,q,dq):
        M = se3.SE3( se3.Quaternion(q[6,0],q[3,0],q[4,0],q[5,0]).matrix(), q[:3])
        dM = exp(dq[:6])
        M = M*dM
        q[:3] = M.translation
        q[3:7] = se3.Quaternion(M.rotation).coeffs()
        q[7:] += dq[6:]

    @property
    def nq(self):
        return self.model.nq
    @property
    def nv(self):
        return self.model.nv

    def com(self,q):
        return se3.centerOfMass(self.model,self.data,q)
    def Jcom(self,q):
        return se3.jacobianCenterOfMass(self.model,self.data,q)

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

    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self,name):
        return [ i for i,n in enumerate(self.model.names) if n == name ][0]

    # --- VIEWER ---
    # For each joint/body index, returns the corresponding name of the node in Gepetto-viewer. 
    def viewerNodeNames(self,index):
        assert( self.model.hasVisual[index] )
        return self.viewerRootNodeName+'/'+self.model.bodyNames[index]

    def initDisplay(self,viewerRootNodeName = "world/pinocchio", loadModel = False):
        import gepetto.corbaserver
        try:
            self.viewer=gepetto.corbaserver.Client()
            self.viewerRootNodeName = viewerRootNodeName
            if loadModel:
                self.loadDisplayModel(viewerRootNodeName)
        except:
            if 'viewer' in self.__dict__: del self.viewer
            print "Error while starting the viewer client. "
            print "Check wheter gepetto-viewer is properly started"

    # Create the scene displaying the robot meshes in Gepetto-viewer.
    def loadDisplayModel(self, nodeName, windowName = "pinocchio", meshDir = None):
        import os
        if not self.viewer.gui.createWindow (windowName):
            print "Warning: window '"+windowName+"' already created. Cannot (re-)load the model."
            return
        if not meshDir: meshDir = os.path.dirname(self.modelFileName)+"/"
        self.viewer.gui.createSceneWithFloor("world")
        self.viewer.gui.addSceneToWindow("world",windowName)
        self.viewer.gui.addURDF(nodeName,
                                self.modelFileName,
                                meshDir)

    # Display in gepetto-view the robot at configuration q, by placing all the bodies.
    def display(self,q): 
        if 'viewer' not in self.__dict__: return
        # Update the robot geometry.
        se3.kinematics(self.model,self.data,q,self.v0)
        # Iteratively place the robot bodies.
        for i in range(1,self.model.nbody):
            if self.model.hasVisual[i]:
                M = self.data.oMi[i]
                self.viewer.gui.applyConfiguration(self.viewerNodeNames(i),
                                                   utils.se3ToXYZQUAT(M))
        self.viewer.gui.refresh()

__all__ = [ 'RobotWrapper' ]
