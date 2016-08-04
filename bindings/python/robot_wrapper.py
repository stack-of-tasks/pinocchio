#
# Copyright (c) 2015-2016 CNRS
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

import time

import libpinocchio_pywrap as se3
import utils
from explog import exp


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
            if verbose:
                print 'Info: the Geometry Module has not been compiled with Pinocchio. No geometry model and data have been built.'
        else:
            if package_dirs is None:
                self.collision_model = se3.buildGeomFromUrdf(self.model, filename)
                self.visual_model = se3.buildGeomFromUrdf(self.model, filename)
                self.collision_data = se3.GeometryData(self.data, self.collision_model)
            else:
                if not all(isinstance(item, basestring) for item in package_dirs):
                    raise Exception('The list of package directories is wrong. At least one is not a string')
                else:
                    self.collision_model = se3.buildGeomFromUrdf(self.model, filename,
                                                                utils.fromListToVectorOfString(package_dirs), se3.GeometryType.COLLISION)
                    self.visual_model = se3.buildGeomFromUrdf(self.model, filename,
                                                                utils.fromListToVectorOfString(package_dirs), se3.GeometryType.VISUAL)
                    self.collision_data = se3.GeometryData(self.collision_model)

        self.v0 = utils.zero(self.nv)
        self.q0 = utils.zero(self.nq)

    def increment(self, q, dq):
        M = se3.SE3(se3.Quaternion(q[6, 0], q[3, 0], q[4, 0], q[5, 0]).matrix(), q[:3])
        dM = exp(dq[:6])
        M = M * dM
        q[:3] = M.translation
        q[3:7] = se3.Quaternion(M.rotation).coeffs()
        q[7:] += dq[6:]

    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    def com(self, q, v=None, a=None, update_kinematics=True):
        if v is not None:
            if a is None:
                se3.centerOfMass(self.model, self.data, q, v, update_kinematics)
                return self.data.com[0], self.data.vcom[0]
            se3.centerOfMass(self.model, self.data, q, v, a, update_kinematics)
            return self.data.com[0], self.data.vcom[0], self.data.acom[0]
        return se3.centerOfMass(self.model, self.data, q, update_kinematics)

    def Jcom(self, q):
        return se3.jacobianCenterOfMass(self.model, self.data, q)

    def mass(self, q):
        return se3.crba(self.model, self.data, q)

    def biais(self, q, v):
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

    def position(self, q, index, update_geometry=True):
        if update_geometry:
            se3.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[index]

    def velocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            se3.forwardKinematics(self.model, self.data, q, v)
        return self.data.v[index]

    def acceleration(self, q, v, a, index, update_acceleration=True):
        if update_acceleration:
            se3.forwardKinematics(self.model, self.data, q, v, a)
        return self.data.a[index]

    def jacobian(self, q, index, update_geometry=True, local_frame=True):
        return se3.jacobian(self.model, self.data, q, index, local_frame, update_geometry)

    def computeJacobians(self, q):
        return se3.computeJacobians(self.model, self.data, q)

    def updateGeometryPlacements(self, q):
        se3.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data, q)


    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self, name):
        return [i for i, n in enumerate(self.model.names) if n == name][0]

    # --- VIEWER ---
    # For each visual object, returns the corresponding name of the node in Gepetto-viewer.
    def viewerNodeNames(self, visual):
        return self.viewerRootNodeName + '/' + visual.name


    def initDisplay(self, viewerRootNodeName="world/pinocchio", loadModel=False):
        import gepetto.corbaserver
        try:
            self.viewer = gepetto.corbaserver.Client()
            self.viewerRootNodeName = viewerRootNodeName
            if loadModel:
                self.loadDisplayModel(viewerRootNodeName)
        except:
            if 'viewer' in self.__dict__:
                del self.viewer
            print "Error while starting the viewer client. "
            print "Check wheter gepetto-viewer is properly started"

    # Create the scene displaying the robot meshes in gepetto-viewer
    def loadDisplayModel(self, nodeName, windowName="pinocchio"):
        import os
        print "load the model"
        # Open a window for displaying your model.
        try:
            # If the window already exists, do not do anything.
            self.windowID = self.viewer.gui.getWindowID(windowName)
            print "Warning: window '%s' already created. Cannot (re-)load the model." % windowName
        except:
             # Otherwise, create the empty window.
            self.windowID = self.viewer.gui.createWindow(windowName)

        # Start a new "scene" in this window, named "world", with just a floor.
        gui = self.viewer.gui
        scene_l = gui.getSceneList()
        if "world" not in scene_l:
          self.viewer.gui.createScene("world")
        self.viewer.gui.addSceneToWindow("world", self.windowID)

        self.viewer.gui.createGroup(nodeName)

        # iterate over visuals and create the meshes in the viewer
        for visual in self.visual_model.geometryObjects :
            meshName = self.viewerNodeNames(visual) 
            meshPath = visual.mesh_path
            self.viewer.gui.addMesh(meshName, meshPath)

        # Finally, refresh the layout to obtain your first rendering.
        self.viewer.gui.refresh()


    # Display in gepetto-view the robot at configuration q, by placing all the bodies.
    def display(self, q):
        if 'viewer' not in self.__dict__:
            return
        # Update the robot kinematics and geometry.
        self.updateGeometryPlacements(q)


        for visual in self.visual_model.geometryObjects :
            M = self.collision_data.oMg[self.visual_model.getGeometryId(visual.name)]
            pinocchioConf = utils.se3ToXYZQUAT(M)
            viewerConf = utils.XYZQUATToViewerConfiguration(pinocchioConf)
            self.viewer.gui.applyConfiguration(self.viewerNodeNames(visual), viewerConf)

        self.viewer.gui.refresh()

    def play(self, q_trajectory, dt):
        for k in range(q_trajectory.shape[1]):
            t0 = time.time()
            self.display(q_trajectory[:, k])
            t1 = time.time()
            elapsed_time = t1 - t0
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

__all__ = ['RobotWrapper']
