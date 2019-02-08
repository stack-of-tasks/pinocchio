#
# Copyright (c) 2015-2018 CNRS
#

from . import libpinocchio_pywrap as pin
from . import utils
from .deprecation import deprecated

import time
import os
import numpy as np

class RobotWrapper(object):

    @staticmethod
    def BuildFromURDF(filename, package_dirs=None, root_joint=None, verbose=False):
        robot = RobotWrapper()
        robot.initFromURDF(filename, package_dirs, root_joint, verbose)
        return robot

    def initFromURDF(self,filename, package_dirs=None, root_joint=None, verbose=False):
        if root_joint is None:
            model = pin.buildModelFromUrdf(filename)
        else:
            model = pin.buildModelFromUrdf(filename, root_joint)

        if "buildGeomFromUrdf" not in dir(pin):
            collision_model = None
            visual_model = None
            if verbose:
                print('Info: the Geometry Module has not been compiled with Pinocchio. No geometry model and data have been built.')
        else:
            if package_dirs is None:
                self.collision_model = pin.buildGeomFromUrdf(self.model, filename, pin.GeometryType.COLLISION)
                self.visual_model = pin.buildGeomFromUrdf(self.model, filename, pin.GeometryType.VISUAL)
            else:
                if not all(isinstance(item, str) for item in package_dirs):
                    raise Exception('The list of package directories is wrong. At least one is not a string')
                else:
                    collision_model = pin.buildGeomFromUrdf(model, filename,
                                                            utils.fromListToVectorOfString(package_dirs), pin.GeometryType.COLLISION)
                    visual_model = pin.buildGeomFromUrdf(model, filename,
                                                         utils.fromListToVectorOfString(package_dirs), pin.GeometryType.VISUAL)


        RobotWrapper.__init__(self,model=model,collision_model=collision_model,visual_model=visual_model)


    def __init__(self, model = pin.Model(), collision_model = None, visual_model = None, verbose=False):

        self.model = model
        self.data = self.model.createData()

        self.collision_model = collision_model
        self.visual_model = visual_model

        if "buildGeomFromUrdf" not in dir(pin):
            self.collision_data = None
            self.visual_data = None
            if verbose:
                print('Info: the Geometry Module has not been compiled with Pinocchio. No geometry model and data have been built.')
        else:
            if self.collision_model is None:
                self.collision_data = None
            else:
                self.collision_data = pin.GeometryData(self.collision_model)

            if self.visual_model is None:
                self.visual_data = None
            else:
                self.visual_data = pin.GeometryData(self.visual_model)

        self.v0 = utils.zero(self.nv)
        self.q0 = pin.neutral(self.model)

        # Two viewers are supported: meshcat or gepetto-gui. By default, gepetto-gui is to be used.
        self.used_viewer = 'gepetto-gui'

    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    def com(self, q=None, v=None, a=None):
        if q is None:
            pin.centerOfMass(self.model, self.data)
            return data.com[0]
        if v is not None:
            if a is None:
                pin.centerOfMass(self.model, self.data, q, v)
                return self.data.com[0], self.data.vcom[0]
            pin.centerOfMass(self.model, self.data, q, v, a)
            return self.data.com[0], self.data.vcom[0], self.data.acom[0]
        return pin.centerOfMass(self.model, self.data, q)

    def vcom(self, q, v):
        pin.centerOfMass(self.model, self.data, q, v)
        return self.data.vcom[0]

    def acom(self, q, v, a):
        pin.centerOfMass(self.model, self.data, q, v, a)
        return self.data.acom[0]

    def centroidalMomentum(self, q, v):
        pin.ccrba(self.model, self.data, q, v)
        return self.data.hg

    def centroidalMomentumVariation(self, q, v, a):
        pin.dccrba(self.model, self.data, q, v)
        return pin.Force(self.data.Ag*a+self.data.dAg*v)

    def Jcom(self, q):
        return pin.jacobianCenterOfMass(self.model, self.data, q)

    def mass(self, q):
        return pin.crba(self.model, self.data, q)

    def nle(self, q, v):
        return pin.nonLinearEffects(self.model, self.data, q, v)

    @deprecated("This method is now renamed nle. Please use nle instead.")
    def bias(self, q, v):
        return pin.nonLinearEffects(self.model, self.data, q, v)

    def gravity(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, q)

    def forwardKinematics(self, q, v=None, a=None):
        if v is not None:
            if a is not None:
                pin.forwardKinematics(self.model, self.data, q, v, a)
            else:
                pin.forwardKinematics(self.model, self.data, q, v)
        else:
            pin.forwardKinematics(self.model, self.data, q)

    @deprecated("This method is now renamed placement. Please use placement instead.")
    def position(self, q, index, update_kinematics=True):
        return self.placement(q, index, update_kinematics)

    def placement(self, q, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[index]

    def velocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v)
        return self.data.v[index]

    def acceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return self.data.a[index]

    @deprecated("This method is now renamed framePlacement. Please use framePlacement instead.")
    def framePosition(self, q, index, update_kinematics=True):
        return self.framePlacement(q, index, update_kinematics)

    def framePlacement(self, q, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q)
        return pin.updateFramePlacement(self.model, self.data, index)

    def frameVelocity(self, q, v, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v)
        return pin.getFrameVelocity(self.model, self.data, index)

    def frameAcceleration(self, q, v, a, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return pin.getFrameAcceleration(self.model, self.data, index)

    def frameClassicAcceleration(self, index):
        v = pin.getFrameVelocity(self.model, self.data, index)
        a = pin.getFrameAcceleration(self.model, self.data, index)
        a.linear += np.cross(v.angular, v.linear, axis=0)
        return a;

    @deprecated("This method is now deprecated. Please use jointJacobian instead. It will be removed in release 1.4.0 of Pinocchio.")
    def jacobian(self, q, index, update_kinematics=True, local_frame=True):
        if local_frame:
            return pin.jointJacobian(self.model, self.data, q, index, pin.ReferenceFrame.LOCAL, update_kinematics)
        else:
            return pin.jointJacobian(self.model, self.data, q, index, pin.ReferenceFrame.WORLD, update_kinematics)

    def jointJacobian(self, q, index, rf_frame=pin.ReferenceFrame.LOCAL, update_kinematics=True):
        return pin.jointJacobian(self.model, self.data, q, index, rf_frame, update_kinematics)

    def getJointJacobian(self, index, rf_frame=pin.ReferenceFrame.LOCAL):
        return pin.getFrameJacobian(self.model, self.data, index, rf_frame)

    @deprecated("This method is now deprecated. Please use computeJointJacobians instead. It will be removed in release 1.4.0 of Pinocchio.")
    def computeJacobians(self, q):
        return pin.computeJointJacobians(self.model, self.data, q)

    def computeJointJacobians(self, q):
        return pin.computeJointJacobians(self.model, self.data, q)

    def updateGeometryPlacements(self, q=None, visual=False):
        if visual:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data

        if q is not None:
            pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data, q)
        else:
            pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)

    @deprecated("This method is now renamed framesForwardKinematics. Please use framesForwardKinematics instead.")
    def framesKinematics(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)

    def framesForwardKinematics(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)

    '''
        It computes the Jacobian of frame given by its id (frame_id) either expressed in the
        local coordinate frame or in the world coordinate frame.
    '''
    def getFrameJacobian(self, frame_id, rf_frame=pin.ReferenceFrame.LOCAL):
        return pin.getFrameJacobian(self.model, self.data, frame_id, rf_frame)

    '''
        Similar to getFrameJacobian but it also calls before pin.computeJointJacobians and
        pin.updateFramePlacements to update internal value of self.data related to frames.
    '''
    def frameJacobian(self, q, frame_id, rf_frame=pin.ReferenceFrame.LOCAL):
        return pin.frameJacobian(self.model, self.data, q, frame_id, rf_frame)


    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self, name):
        return [i for i, n in enumerate(self.model.names) if n == name][0]

    # --- VIEWER ---
    # For each geometry object, returns the corresponding name of the node in Gepetto-viewer.
    def getViewerNodeName(self, geometry_object, geometry_type):
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + '/' + geometry_object.name


    def initDisplay(self, windowName="python-pinocchio", sceneName="world", loadModel=False):
        """
        Init gepetto-viewer by loading the gui and creating a window.
        """
        # Set viewer to use to gepetto-gui.
        self.used_viewer = 'gepetto-gui'
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

    def initMeshcatDisplay(self, meshcat_visualizer, robot_name = "pinocchio", robot_color = None):
        """ Load the robot in a MeshCat viewer.
        Parameters:
            visualizer: the meshcat.Visualizer instance to use.
            robot_name: name to give to the robot in the viewer
            robot_color: optional, color to give to the robot. This overwrites the color present in the urdf.
                         Format is a list of four RGBA floats (between 0 and 1)
        """
        import meshcat.geometry
        # Set viewer to use to gepetto-gui.
        self.used_viewer = 'meshcat'
        self.meshcat_viewer = meshcat_visualizer
        self.viewerRootNodeName = robot_name

        # Load robot meshes in MeshCat
        for visual in self.visual_model.geometryObjects:
            viewer_name = self.viewerRootNodeName + visual.name
            if visual.meshPath == "":
                raise IOError("Visual mesh file not found for link {}.".format(visual.name))
            # Get file type from filename extension.
            _, file_extension = os.path.splitext(visual.meshPath)
            if file_extension.lower() == ".dae":
                obj = meshcat.geometry.DaeMeshGeometry.from_file(visual.meshPath)
            elif file_extension.lower() == ".obj":
                obj = meshcat.geometry.ObjMeshGeometry.from_file(visual.meshPath)
            elif file_extension.lower() == ".stl":
                obj = meshcat.geometry.StlMeshGeometry.from_file(visual.meshPath)
            else:
                raise ImportError("Unknown mesh file format: {}.".format(visual.meshPath))
            material = meshcat.geometry.MeshPhongMaterial()
            # Set material color from URDF, converting for triplet of doubles to a single int.
            if robot_color is None:
                meshColor = visual.meshColor
            else:
                meshColor = robot_color
            material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)
            # Add transparency, if needed.
            if float(meshColor[3]) != 1.0:
                material.transparent = True
                material.opacity = float(meshColor[3])
            self.meshcat_viewer[viewer_name].set_object(obj, material)

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
            loadDisplayGeometryObject(collision,pin.GeometryType.COLLISION)
        self.displayCollisions(False)

        for visual in self.visual_model.geometryObjects:
            loadDisplayGeometryObject(visual,pin.GeometryType.VISUAL)
        self.displayVisuals(True)

        # Finally, refresh the layout to obtain your first rendering.
        gui.refresh()

    # Display the robot at configuration q in the viewer (meshcat or gepetto-gui), by placing all the bodies.
    def display(self, q):
        if self.used_viewer == 'gepetto-gui':
            if 'viewer' not in self.__dict__:
                return

            gui = self.viewer.gui
            # Update the robot kinematics and geometry.
            self.forwardKinematics(q)

            if self.display_collisions:
                gui.applyConfigurations (
                        [ self.getViewerNodeName(collision,pin.GeometryType.COLLISION) for collision in self.collision_model.geometryObjects ],
                        [ pin.se3ToXYZQUATtuple(self.collision_data.oMg[self.collision_model.getGeometryId(collision.name)]) for collision in self.collision_model.geometryObjects ]
                        )

            if self.display_visuals:
                self.updateGeometryPlacements(visual=True)
                gui.applyConfigurations (
                        [ self.getViewerNodeName(visual,pin.GeometryType.VISUAL) for visual in self.visual_model.geometryObjects ],
                        [ pin.se3ToXYZQUATtuple(self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]) for visual in self.visual_model.geometryObjects ]
                        )

            gui.refresh()
        elif self.used_viewer == 'meshcat':
            # Update the robot kinematics and geometry.
            self.forwardKinematics(q)
            self.updateGeometryPlacements(visual=True)
            for visual in self.visual_model.geometryObjects:
                # Get mesh pose.
                M = self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]
                # Update viewer configuration.
                self.meshcat_viewer[self.viewerRootNodeName + visual.name].set_transform(np.array(M.homogeneous))

    def displayCollisions(self,visibility):
        gui = self.viewer.gui
        self.display_collisions = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for collision in self.collision_model.geometryObjects:
            nodeName = self.getViewerNodeName(collision,pin.GeometryType.COLLISION)
            gui.setVisibility(nodeName,visibility_mode)

    def displayVisuals(self,visibility):
        gui = self.viewer.gui
        self.display_visuals = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for visual in self.visual_model.geometryObjects:
            nodeName = self.getViewerNodeName(visual,pin.GeometryType.VISUAL)
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
