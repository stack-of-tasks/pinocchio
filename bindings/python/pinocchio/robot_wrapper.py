#
# Copyright (c) 2015-2020 CNRS INRIA
#

from . import pinocchio_pywrap_default as pin
from . import utils
from .shortcuts import (
    buildModelsFromMJCF,
    buildModelsFromSdf,
    buildModelsFromUrdf,
    createDatas,
)


class RobotWrapper:
    @staticmethod
    def BuildFromURDF(filename, *args, **kwargs):
        robot = RobotWrapper()

        robot.initFromURDF(filename, *args, **kwargs)

        return robot

    def initFromURDF(self, filename, *args, **kwargs):
        model, collision_model, visual_model = buildModelsFromUrdf(
            filename, *args, **kwargs
        )

        RobotWrapper.__init__(
            self,
            model=model,
            collision_model=collision_model,
            visual_model=visual_model,
        )

    @staticmethod
    def BuildFromSDF(filename, *args, **kwargs):
        robot = RobotWrapper()
        robot.initFromSDF(filename, *args, **kwargs)
        return robot

    def initFromSDF(self, filename, *args, **kwargs):
        model, constraint_models, collision_model, visual_model = buildModelsFromSdf(
            filename, *args, **kwargs
        )

        RobotWrapper.__init__(
            self,
            model=model,
            collision_model=collision_model,
            visual_model=visual_model,
        )
        self.constraint_models = constraint_models

    @staticmethod
    def BuildFromMJCF(filename, *args, **kwargs):
        robot = RobotWrapper()
        robot.initFromMJCF(filename, *args, **kwargs)

        return robot

    def initFromMJCF(self, filename, *args, **kwargs):
        model, collision_model, visual_model = buildModelsFromMJCF(
            filename, *args, **kwargs
        )

        RobotWrapper.__init__(
            self,
            model=model,
            collision_model=collision_model,
            visual_model=visual_model,
        )

    def __init__(
        self, model=pin.Model(), collision_model=None, visual_model=None, verbose=False
    ):
        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.data, self.collision_data, self.visual_data = createDatas(
            model, collision_model, visual_model
        )

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
        return pin.computeCentroidalMomentum(self.model, self.data, q, v)

    def centroidalMap(self, q):
        """
        Computes the centroidal momentum matrix which maps from the joint velocity
        vector to the centroidal momentum expressed around the center of mass.
        """
        return pin.computeCentroidalMap(self.model, self.data, q)

    def centroidal(self, q, v):
        """
        Computes all the quantities related to the centroidal dynamics (hg, Ag and Ig),
        corresponding to the centroidal momentum, the centroidal map and the centroidal
        rigid inertia.
        """
        pin.ccrba(self.model, self.data, q, v)
        return (self.data.hg, self.data.Ag, self.data.Ig)

    def centroidalMomentumVariation(self, q, v, a):
        return pin.computeCentroidalMomentumTimeVariation(
            self.model, self.data, q, v, a
        )

    def Jcom(self, q):
        return pin.jacobianCenterOfMass(self.model, self.data, q)

    def mass(self, q):
        return pin.crba(self.model, self.data, q)

    def nle(self, q, v):
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

    def placement(self, q, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[index]

    def velocity(
        self,
        q,
        v,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v)
        return pin.getVelocity(self.model, self.data, index, reference_frame)

    def acceleration(
        self,
        q,
        v,
        a,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return pin.getAcceleration(self.model, self.data, index, reference_frame)

    def classicalAcceleration(
        self,
        q,
        v,
        a,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return pin.getClassicalAcceleration(
            self.model, self.data, index, reference_frame
        )

    def framePlacement(self, q, index, update_kinematics=True):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q)
        return pin.updateFramePlacement(self.model, self.data, index)

    def frameVelocity(
        self,
        q,
        v,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v)
        return pin.getFrameVelocity(self.model, self.data, index, reference_frame)

    def frameAcceleration(
        self,
        q,
        v,
        a,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return pin.getFrameAcceleration(self.model, self.data, index, reference_frame)

    def frameClassicalAcceleration(
        self,
        q,
        v,
        a,
        index,
        update_kinematics=True,
        reference_frame=pin.ReferenceFrame.LOCAL,
    ):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v, a)
        return pin.getFrameClassicalAcceleration(
            self.model, self.data, index, reference_frame
        )

    def computeJointJacobian(self, q, index):
        return pin.computeJointJacobian(self.model, self.data, q, index)

    def getJointJacobian(self, index, rf_frame=pin.ReferenceFrame.LOCAL):
        return pin.getJointJacobian(self.model, self.data, index, rf_frame)

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
            pin.updateGeometryPlacements(
                self.model, self.data, geom_model, geom_data, q
            )
        else:
            pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)

    def framesForwardKinematics(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)

    def buildReducedRobot(self, list_of_joints_to_lock, reference_configuration=None):
        """
        Build a reduced robot model given a list of joints to lock.
        Parameters:
        \tlist_of_joints_to_lock: list of joint indexes/names to lock.
        \treference_configuration: reference configuration to compute the
        placement of the lock joints. If not provided, reference_configuration
        defaults to the robot's neutral configuration.

        Returns: a new robot model.
        """

        # if joint to lock is a string, try to find its index
        lockjoints_idx = []
        for jnt in list_of_joints_to_lock:
            idx = jnt
            if isinstance(jnt, str):
                idx = self.model.getJointId(jnt)
            lockjoints_idx.append(idx)

        if reference_configuration is None:
            reference_configuration = pin.neutral(self.model)

        model, geom_models = pin.buildReducedModel(
            model=self.model,
            list_of_geom_models=[self.visual_model, self.collision_model],
            list_of_joints_to_lock=lockjoints_idx,
            reference_configuration=reference_configuration,
        )

        return RobotWrapper(
            model=model, visual_model=geom_models[0], collision_model=geom_models[1]
        )

    def getFrameJacobian(self, frame_id, rf_frame=pin.ReferenceFrame.LOCAL):
        """
        It computes the Jacobian of frame given by its id (frame_id) either expressed in
        the local coordinate frame or in the world coordinate frame.
        """
        return pin.getFrameJacobian(self.model, self.data, frame_id, rf_frame)

    def computeFrameJacobian(self, q, frame_id):
        """
        Similar to getFrameJacobian but does not need pin.computeJointJacobians and
        pin.updateFramePlacements to update internal value of self.data related to
        frames.
        """
        return pin.computeFrameJacobian(self.model, self.data, q, frame_id)

    def rebuildData(self):
        """Re-build the data objects. Needed if the models were modified.
        Warning: this will delete any information stored in all data objects."""
        data, collision_data, visual_data = createDatas(
            self.model, self.collision_model, self.visual_model
        )
        if self.viz is not None:
            if (
                id(self.data) == id(self.viz.data)
                and id(self.collision_data) == id(self.viz.collision_data)
                and id(self.visual_data) == id(self.viz.visual_data)
            ):
                self.viz.data = data
                self.viz.collision_data = collision_data
                self.viz.visual_data = visual_data
            else:
                self.viz.rebuildData()
        self.data = data
        self.collision_data = collision_data
        self.visual_data = visual_data

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
        """
        Set the visualizer. If init is True, the visualizer is initialized with this
        wrapper's models.  If copy_models is also True, the models are copied.
        Otherwise, they are simply kept as a reference.
        """
        if init:
            visualizer.__init__(
                self.model, self.collision_model, self.visual_model, copy_models
            )
        self.viz = visualizer

    def getViewerNodeName(self, geometry_object, geometry_type):
        """
        For each geometry object, returns the corresponding name of the node in the
        display.
        """
        return self.viz.getViewerNodeName(geometry_object, geometry_type)

    def initViewer(self, share_data=True, *args, **kwargs):
        """Init the viewer"""
        # Set viewer to use to MeshCat.
        if self.viz is None:
            from .visualize import Visualizer

            data, collision_data, visual_data = None, None, None
            if share_data:
                data = self.data
                collision_data = self.collision_data
                visual_data = self.visual_data
            self.viz = Visualizer.default()(
                self.model,
                self.collision_model,
                self.visual_model,
                not share_data,
                data,
                collision_data,
                visual_data,
            )

        self.viz.initViewer(*args, **kwargs)

    def loadViewerModel(self, *args, **kwargs):
        """Create the scene displaying the robot meshes in MeshCat"""
        self.viz.loadViewerModel(*args, **kwargs)

    def display(self, q):
        """
        Display the robot at configuration q in the viewer by placing all the bodies.
        """
        self.viz.display(q)

    def displayCollisions(self, visibility):
        """Set whether to diplay collision objects or not"""
        self.viz.displayCollisions(visibility)

    def displayVisuals(self, visibility):
        """Set whether to diplay visual objects or not"""
        self.viz.displayVisuals(visibility)

    def play(self, q_trajectory, dt):
        """Play a trajectory with given time step"""
        self.viz.play(q_trajectory, dt)


__all__ = ["RobotWrapper"]
