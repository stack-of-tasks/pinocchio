#
# Copyright (c) 2015-2017 CNRS
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

import libpinocchio_pywrap as se3
import utils
import time
import os

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
                if not all(isinstance(item, basestring) for item in package_dirs):
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

    def increment(self, q, dq):
        q_next = se3.integrate(self.model,q,dq)
        q[:] = q_next[:]

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

    def position(self, q, index, update_kinematics=True):
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

    def framePosition(self, q, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q)
        frame = self.model.frames[index]
        parentPos = self.data.oMi[frame.parent]
        return parentPos.act(frame.placement)

    def frameVelocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v)
        frame = self.model.frames[index]
        parentJointVel = self.data.v[frame.parent]
        return frame.placement.actInv(parentJointVel)

    def frameAcceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v, a)
        frame = self.model.frames[index]
        parentJointAcc = self.data.a[frame.parent]
        return frame.placement.actInv(parentJointAcc)

    def frameClassicAcceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v, a)      
        f = self.model.frames[index]
        af = f.placement.actInv(self.data.a[f.parent])
        vf = f.placement.actInv(self.data.v[f.parent])
        af.linear += np.cross(vf.angular.T, vf.linear.T).T
        return af;

    def jacobian(self, q, index, update_kinematics=True, local_frame=True):
        return se3.jacobian(self.model, self.data, q, index, local_frame, update_kinematics)

    def computeJacobians(self, q):
        return se3.computeJacobians(self.model, self.data, q)

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


    def framesKinematics(self, q): 
        se3.framesKinematics(self.model, self.data, q)
   
    ''' Call computeJacobians if update_geometry is true. If not, user should call computeJacobians first.
    Then call getJacobian and return the resulted jacobian matrix. Attention: if update_geometry is true, 
    the function computes all the jacobians of the model. It is therefore outrageously costly wrt a 
    dedicated call. Use only with update_geometry for prototyping.
    '''
    def frameJacobian(self, q, index, update_geometry=True, local_frame=True):
        return se3.frameJacobian(self.model, self.data, q, index, local_frame, update_geometry)

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
            from rpy import npToTuple
            
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
                M = self.visual_data.oMg[self.collision_model.getGeometryId(collision.name)]
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
