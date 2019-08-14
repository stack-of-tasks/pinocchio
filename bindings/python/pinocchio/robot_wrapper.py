#
# Copyright (c) 2015-2019 CNRS
#

from . import libpinocchio_pywrap as pin
from . import utils
from .deprecation import deprecated
from .shortcuts import buildModelsFromUrdf, createDatas

import numpy as np

class RobotWrapper(object):

    @staticmethod
    def BuildFromURDF(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
        robot = RobotWrapper()
        robot.initFromURDF(filename, package_dirs, root_joint, verbose, meshLoader)
        return robot

    def initFromURDF(self,filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
        model, collision_model, visual_model = buildModelsFromUrdf(filename, package_dirs, root_joint, verbose, meshLoader)
        RobotWrapper.__init__(self,model=model,collision_model=collision_model,visual_model=visual_model)

    def __init__(self, model = pin.Model(), collision_model = None, visual_model = None, verbose=False):

        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.data, self.collision_data, self.visual_data = createDatas(model,collision_model,visual_model)

        self.v0 = utils.zero(self.nv)
        self.q0 = pin.neutral(self.model)

        self.viz = None

    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    def com(self, q=None, v=None, a=None):
        if q is None:
            pin.centerOfMass(self.model, self.data)
            return self.data.com[0]
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

    def jointJacobian(self, q, index, *args):
        if len(args)==0:
            return pin.jointJacobian(self.model, self.data, q, index)
        else: # use deprecated signature (19 Feb 2019)
            update_kinematics = True if len(args)==1 else args[1]
            rf = args[0]
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
        Similar to getFrameJacobian but does not need pin.computeJointJacobians and
        pin.updateFramePlacements to update internal value of self.data related to frames.
    '''
    def frameJacobian(self, q, frame_id, rf_frame=None):
        if rf_frame: # use deprecated signature (19 Feb 2019)
            return pin.frameJacobian(self.model, self.data, q, frame_id, rf_frame)
        else: # use normal signature
            return pin.frameJacobian(self.model, self.data, q, frame_id)


    # --- ACCESS TO NAMES ----
    # Return the index of the joint whose name is given in argument.
    def index(self, name):
        return self.model.getJointId(name)

    # --- VIEWER ---

    # for backwards compatibility
    @property
    def viewer(self):
        return self.viz.viewer

    def setVisualizer(self, visualizer, init=True, copy_models=False):
        """Set the visualizer. If init is True, the visualizer is initialized with this wrapper's models.
        If copy_models is also True, the models are copied. Otherwise, they are simply kept as a reference.
        """
        if init:
            visualizer.__init__(self.model, self.collision_model, self.visual_model, copy_models)
        self.viz = visualizer

    def getViewerNodeName(self, geometry_object, geometry_type):
        """For each geometry object, returns the corresponding name of the node in the display."""
        return self.viz.getViewerNodeName(geometry_object, geometry_type)

    def initViewer(self, *args, **kwargs):
        """Init the viewer"""
        # Set viewer to use to gepetto-gui.
        if self.viz is None:
            from .visualize import GepettoVisualizer
            self.viz = GepettoVisualizer(self.model, self.collision_model, self.visual_model)

        self.viz.initViewer(*args, **kwargs)

    @deprecated("Use initViewer")
    def initDisplay(self, windowName="python-pinocchio", sceneName="world", loadModel=False):
        self.initViewer(windowName=windowName, sceneName=sceneName, loadModel=loadModel)

    @deprecated("You should manually set the visualizer, initialize it, and load the model.")
    def initMeshcatDisplay(self, meshcat_visualizer, robot_name = "pinocchio", robot_color = None):
        """ Load the robot in a Meshcat viewer.
        Parameters:
            visualizer: the meshcat.Visualizer instance to use.
            robot_name: name to give to the robot in the viewer
            robot_color: optional, color to give to the robot. This overwrites the color present in the urdf.
                         Format is a list of four RGBA floats (between 0 and 1)
        """
        from .visualize import MeshcatVisualizer
        self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        self.viz.initViewer(meshcat_visualizer)
        self.viz.loadViewerModel(rootNodeName=robot_name, color=robot_color)

    def loadViewerModel(self, *args, **kwargs):
        """Create the scene displaying the robot meshes in gepetto-viewer"""
        self.viz.loadViewerModel(*args, **kwargs)

    @deprecated("Use loadViewerModel")
    def loadDisplayModel(self, rootNodeName="pinocchio"):
        """Create the scene displaying the robot meshes in gepetto-viewer"""
        self.loadViewerModel(rootNodeName=rootNodeName)

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        self.viz.display(q)

    def displayCollisions(self,visibility):
        """Set whether to diplay collision objects or not"""
        self.viz.displayCollisions(visibility)

    def displayVisuals(self,visibility):
        """Set whether to diplay visual objects or not"""
        self.viz.displayVisuals(visibility)

    def play(self, q_trajectory, dt):
        """Play a trajectory with given time step"""
        self.viz.play(q_trajectory, dt)

__all__ = ['RobotWrapper']
