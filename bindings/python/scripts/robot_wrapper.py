#
# Copyright (c) 2015-2018 CNRS
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

from . import libpinocchio_pywrap as se3
from . import utils
from .deprecation import deprecated

import time
import os
import numpy as np

class RobotWrapper(object):

    def __init__(self, filename, package_dirs=None, root_joint=None, verbose=False):
        if root_joint is None:
            self.model = se3.buildModelFromUrdf(filename)
        else:
            self.model = se3.buildModelFromUrdf(filename, root_joint)

        self.data = self.model.createData()
        self.model_filename = filename

        if "buildGeomFromUrdf" not in dir(se3):
            self.collision_model = None
            self.visual_model = None
            self.collision_data = None
            self.visual_data = None
            if verbose:
                print('Info: the Geometry Module has not been compiled with Pinocchio. No geometry model and data have been built.')
        else:
            if package_dirs is None:
                self.collision_model = se3.buildGeomFromUrdf(self.model, filename, se3.GeometryType.COLLISION)
                self.visual_model = se3.buildGeomFromUrdf(self.model, filename, se3.GeometryType.VISUAL)
            else:
                if not all(isinstance(item, str) for item in package_dirs):
                    raise Exception('The list of package directories is wrong. At least one is not a string')
                else:
                    self.collision_model = se3.buildGeomFromUrdf(self.model, filename,
                                                                utils.fromListToVectorOfString(package_dirs), se3.GeometryType.COLLISION)
                    self.visual_model = se3.buildGeomFromUrdf(self.model, filename,
                                                                utils.fromListToVectorOfString(package_dirs), se3.GeometryType.VISUAL)
            self.collision_data = se3.GeometryData(self.collision_model)
            self.visual_data = se3.GeometryData(self.visual_model)

        self.v0 = utils.zero(self.nv)
        self.q0 = self.model.neutralConfiguration

    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    def com(self, q=None, v=None, a=None):
        if q is None:
            se3.centerOfMass(self.model, self.data)
            return data.com[0]
        if v is not None:
            if a is None:
                se3.centerOfMass(self.model, self.data, q, v)
                return self.data.com[0], self.data.vcom[0]
            se3.centerOfMass(self.model, self.data, q, v, a)
            return self.data.com[0], self.data.vcom[0], self.data.acom[0]
        return se3.centerOfMass(self.model, self.data, q)

    def vcom(self, q, v):
        se3.centerOfMass(self.model, self.data, q, v)
        return self.data.vcom[0]

    def acom(self, q, v, a):
        se3.centerOfMass(self.model, self.data, q, v, a)
        return self.data.acom[0]

    def centroidalMomentum(self, q, v):
        se3.ccrba(self.model, self.data, q, v)
        return self.data.hg

    def centroidalMomentumVariation(self, q, v, a):
        se3.dccrba(self.model, self.data, q, v)
        return se3.Force(self.data.Ag*a+self.data.dAg*v)

    def Jcom(self, q):
        return se3.jacobianCenterOfMass(self.model, self.data, q)

    def mass(self, q):
        return se3.crba(self.model, self.data, q)

    def bias(self, q, v):
        return se3.nle(self.model, self.data, q, v)

    def gravity(self, q):
        return se3.rnea(self.model, self.data, q, self.v0, self.v0)

    def forwardKinematics(self, q, v=None, a=None):
        if v is not None:
            if a is not None:
                se3.forwardKinematics(self.model, self.data, q, v, a)
            else:
                se3.forwardKinematics(self.model, self.data, q, v)
        else:
            se3.forwardKinematics(self.model, self.data, q)

    @deprecated("This method is now renamed placement. Please use placement instead.")
    def position(self, q, index, update_kinematics=True):
        return self.placement(q, index, update_kinematics)

    def placement(self, q, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[index]

    def velocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v)
        return self.data.v[index]

    def acceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v, a)
        return self.data.a[index]

    @deprecated("This method is now renamed framePlacement. Please use framePlacement instead.")
    def framePosition(self, q, index, update_kinematics=True):
        return self.framePlacement(q, index, update_kinematics)

    def framePlacement(self, q, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q)
        return se3.updateFramePlacement(self.model, self.data, index)

    def frameVelocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v)
        return se3.getFrameVelocity(self.model, self.data, index)

    def frameAcceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v, a)
        return se3.getFrameAcceleration(self.model, self.data, index)

    def frameClassicAcceleration(self, index):
        v = se3.getFrameVelocity(self.model, self.data, index)
        a = se3.getFrameAcceleration(self.model, self.data, index)
        a.linear += np.cross(v.angular, v.linear, axis=0)
        return a;

    @deprecated("This method is now deprecated. Please use jointJacobian instead. It will be removed in release 1.4.0 of Pinocchio.")
    def jacobian(self, q, index, update_kinematics=True, local_frame=True):
        if local_frame:
            return se3.jointJacobian(self.model, self.data, q, index, se3.ReferenceFrame.LOCAL, update_kinematics)
        else:
            return se3.jointJacobian(self.model, self.data, q, index, se3.ReferenceFrame.WORLD, update_kinematics)

    def jointJacobian(self, q, index, update_kinematics=True, local_frame=True):
        if local_frame:
            return se3.jointJacobian(self.model, self.data, q, index, se3.ReferenceFrame.LOCAL, update_kinematics)
        else:
            return se3.jointJacobian(self.model, self.data, q, index, se3.ReferenceFrame.WORLD, update_kinematics)

    @deprecated("This method is now deprecated. Please use computeJointJacobians instead. It will be removed in release 1.4.0 of Pinocchio.")
    def computeJacobians(self, q):
        return se3.computeJointJacobians(self.model, self.data, q)

    def computeJointJacobians(self, q):
        return se3.computeJointJacobians(self.model, self.data, q)

    def updateGeometryPlacements(self, q=None, visual=False):
        if visual:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data

        if q is not None:
            se3.updateGeometryPlacements(self.model, self.data, geom_model, geom_data, q)
        else:
            se3.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)

    @deprecated("This method is now renamed framesForwardKinematics. Please use framesForwardKinematics instead.")
    def framesKinematics(self, q): 
        se3.framesForwardKinematics(self.model, self.data, q)

    def framesForwardKinematics(self, q): 
        se3.framesForwardKinematics(self.model, self.data, q)
    
    '''
        It computes the Jacobian of frame given by its id (frame_id) either expressed in the
        local coordinate frame or in the world coordinate frame.
    '''
    def getFrameJacobian(self, frame_id, rf_frame):
        return se3.getFrameJacobian(self.model, self.data, frame_id, rf_frame)

    '''
        Similar to getFrameJacobian but it also calls before se3.computeJointJacobians and
        se3.framesForwardKinematics to update internal value of self.data related to frames.
    '''
    def frameJacobian(self, q, frame_id, rf_frame):
        return se3.frameJacobian(self.model, self.data, q, frame_id, rf_frame)

  
    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self, name):
        return [i for i, n in enumerate(self.model.names) if n == name][0]

    # --- VIEWER ---
    # For each geometry object, returns the corresponding name of the node in Gepetto-viewer.
    def getViewerNodeName(self, geometry_object, geometry_type):
        if geometry_type is se3.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is se3.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + '/' + geometry_object.name


    def initDisplay(self, windowName="python-pinocchio", sceneName="world", loadModel=False):
        """
        Init gepetto-viewer by loading the gui and creating a window.
        """
        import gepetto.corbaserver
        try:
            self.viewer = gepetto.corbaserver.Client()
            gui = self.viewer.gui

            # Create window
            window_l = gui.getWindowList()
            if not windowName in window_l:
                self.windowID = self.viewer.gui.createWindow(windowName)
            else:
                self.windowID = self.viewer.gui.getWindowID(windowName)
            
            # Create scene if needed
            scene_l = gui.getSceneList()
            if sceneName not in scene_l:
                gui.createScene(sceneName)
            self.sceneName = sceneName
            gui.addSceneToWindow(sceneName, self.windowID)
            
            if loadModel:
                self.loadDisplayModel()
        except:
            print("Error while starting the viewer client. ")
            print("Check wheter gepetto-viewer is properly started")

    # Create the scene displaying the robot meshes in gepetto-viewer
    def loadDisplayModel(self, rootNodeName="pinocchio"):
    
        def loadDisplayGeometryObject(geometry_object,geometry_type):
            from .rpy import npToTuple
            
            meshName = self.getViewerNodeName(geometry_object,geometry_type)
            meshPath = geometry_object.meshPath
            meshTexturePath = geometry_object.meshTexturePath
            meshScale = geometry_object.meshScale
            meshColor = geometry_object.meshColor
            if gui.addMesh(meshName, meshPath):
                gui.setScale(meshName, npToTuple(meshScale))
                if geometry_object.overrideMaterial:
                    gui.setColor(meshName, npToTuple(meshColor))
                    if meshTexturePath is not '':
                        gui.setTexture(meshName, meshTexturePath)


        # Start a new "scene" in this window, named "world", with just a floor.
        gui = self.viewer.gui
        self.viewerRootNodeName = self.sceneName + "/" + rootNodeName

        if not gui.nodeExists(self.viewerRootNodeName):
            gui.createGroup(self.viewerRootNodeName)

        self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"
        if not gui.nodeExists(self.viewerCollisionGroupName):
            gui.createGroup(self.viewerCollisionGroupName)

        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"
        if not gui.nodeExists(self.viewerVisualGroupName):
            gui.createGroup(self.viewerVisualGroupName)

        # iterate over visuals and create the meshes in the viewer
        for collision in self.collision_model.geometryObjects:
            loadDisplayGeometryObject(collision,se3.GeometryType.COLLISION)
        self.displayCollisions(False)

        for visual in self.visual_model.geometryObjects:
            loadDisplayGeometryObject(visual,se3.GeometryType.VISUAL)
        self.displayVisuals(True)

        # Finally, refresh the layout to obtain your first rendering.
        gui.refresh()

    # Display in gepetto-view the robot at configuration q, by placing all the bodies.
    def display(self, q):
        if 'viewer' not in self.__dict__:
            return

        gui = self.viewer.gui
        # Update the robot kinematics and geometry.
        self.forwardKinematics(q)

        if self.display_collisions:
            self.updateGeometryPlacements(visual=False)
            for collision in self.collision_model.geometryObjects:
                M = self.collision_data.oMg[self.collision_model.getGeometryId(collision.name)]
                conf = utils.se3ToXYZQUAT(M)
                gui.applyConfiguration(self.getViewerNodeName(collision,se3.GeometryType.COLLISION), conf)

        if self.display_visuals:
            self.updateGeometryPlacements(visual=True)
            for visual in self.visual_model.geometryObjects:
                M = self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]
                conf = utils.se3ToXYZQUAT(M)
                gui.applyConfiguration(self.getViewerNodeName(visual,se3.GeometryType.VISUAL), conf)

        gui.refresh()

    def displayCollisions(self,visibility):
        gui = self.viewer.gui
        self.display_collisions = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for collision in self.collision_model.geometryObjects:
            nodeName = self.getViewerNodeName(collision,se3.GeometryType.COLLISION)
            gui.setVisibility(nodeName,visibility_mode)

    def displayVisuals(self,visibility):
        gui = self.viewer.gui
        self.display_visuals = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for visual in self.visual_model.geometryObjects:
            nodeName = self.getViewerNodeName(visual,se3.GeometryType.VISUAL)
            gui.setVisibility(nodeName,visibility_mode)

    def play(self, q_trajectory, dt):
        for k in range(q_trajectory.shape[1]):
            t0 = time.time()
            self.display(q_trajectory[:, k])
            t1 = time.time()
            elapsed_time = t1 - t0
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

__all__ = ['RobotWrapper']
