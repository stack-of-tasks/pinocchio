#
# Copyright (c) 2015 CNRS
#
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.

import numpy as np
import libpinocchio_pywrap as se3
import utils
from explog import exp
import time

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

    def com(self,*args):
        if len(args) == 3:
            q = args[0]
            v = args[1]
            a = args[2]
            se3.centerOfMassAcceleration(self.model,self.data,q,v,a)
            return self.data.com_pos(0), self.data.com_vel(0), self.data.com_acc(0)
        return se3.centerOfMass(self.model,self.data,args[0])

    def Jcom(self,q):
        return se3.jacobianCenterOfMass(self.model,self.data,q)

    def mass(self,q):
        return se3.crba(self.model,self.data,q)
    def biais(self,q,v):
        return se3.nle(self.model,self.data,q,v)
    def gravity(self,q):
        return se3.rnea(self.model,self.data,q,self.v0,self.v0)
    
    def geometry(self,q):
        se3.geometry(self.model, self.data, q)
    def kinematics(self,q,v):
        se3.kinematics(self.model, self.data, q, v)
    def dynamics(self,q,v,a):
        se3.dynamics(self.model, self.data, q, v, a)

    def position(self,q,index, update_geometry = True):
        if update_geometry:
            se3.geometry(self.model,self.data,q)

        return self.data.oMi[index]
    def velocity(self,q,v,index, update_kinematics = True):
        if update_kinematics:
            se3.kinematics(self.model,self.data,q,v)

        return self.data.v[index]
    def acceleration(self,q,v,a,index, update_acceleration = True):
        if update_acceleration:
          se3.dynamics(self.model,self.data,q,v,a)
        return self.data.a[index]
    def jacobian(self,q,index, update_geometry = True):
        return se3.jacobian(self.model,self.data,q,index,True,update_geometry)
    def computeJacobians(self,q):
        return se3.computeJacobians(self.model,self.data,q)

    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self,name):
        return [ i for i,n in enumerate(self.model.names) if n == name ][0]

    # --- VIEWER ---
    # For each joint/body index, returns the corresponding name of the node in Gepetto-viewer. 
    def viewerNodeNames(self,index):
        assert( self.model.hasVisual[index] )
        return self.viewerRootNodeName+'/'+self.model.bodyNames[index]

    def viewerFixedNodeNames(self,index):
        assert( self.model.fix_hasVisual[index] )
        return self.viewerRootNodeName+'/'+self.model.fix_bodyNames[index]


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
        

    # Create the scene displaying the robot meshes in gepetto-viewer
    def loadDisplayModel(self, nodeName, windowName = "pinocchio", meshDir = None):
        import os
        try:
          self.windowID = self.viewer.gui.getWindowID (windowName)
          print "Warning: window '"+windowName+"' already created. Cannot (re-)load the model."
          return
        except:
          self.windowID = self.viewer.gui.createWindow (windowName)
          if not meshDir: meshDir = os.path.dirname(self.modelFileName)+"/"
          self.viewer.gui.createSceneWithFloor("world")
          self.viewer.gui.addSceneToWindow("world",self.windowID)
          self.viewer.gui.addURDF(nodeName,
                                  self.modelFileName,
                                  meshDir)

    # Display in gepetto-view the robot at configuration q, by placing all the bodies.
    def display(self,q): 
        if 'viewer' not in self.__dict__: return
        # Update the robot geometry.
        se3.geometry(self.model,self.data,q)
        # Iteratively place the moving robot bodies.
        for i in range(1,self.model.nbody):
            if self.model.hasVisual[i]:
                M = self.data.oMi[i]
                pinocchioConf = utils.se3ToXYZQUAT(M)
                viewerConf = utils.XYZQUATToViewerConfiguration(pinocchioConf)
                self.viewer.gui.applyConfiguration(self.viewerNodeNames(i),
                                                   viewerConf)
        # Iteratively place the fixed robot bodies.                                                   
        for i in range(0,self.model.nFixBody):
            if self.model.fix_hasVisual[i]:
                index_last_movable=self.model.fix_lastMovingParent[i]
                oMlmp = self.data.oMi[index_last_movable]
                lmpMi = self.model.fix_lmpMi[i]
                M     =  oMlmp * lmpMi
                pinocchioConf = utils.se3ToXYZQUAT(M)
                viewerConf = utils.XYZQUATToViewerConfiguration(pinocchioConf)
                self.viewer.gui.applyConfiguration(self.viewerFixedNodeNames(i),viewerConf)
        self.viewer.gui.refresh()

    def play(self,q_trajectory,dt):
        _,N = q_trajectory.shape

        for k in range(N):
            t0 = time.time()
            self.display(q_trajectory[:,k])
            t1 = time.time()
            elapsed_time = t1-t0
            if elapsed_time < dt:
              time.sleep(dt - elapsed_time)

__all__ = [ 'RobotWrapper' ]
