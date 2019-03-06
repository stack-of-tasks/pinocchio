import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import eye

import numpy as np


class MobileRobotWrapper(RobotWrapper):
    '''
    The class models a mobile robot with UR5 arm, feature 3+6 DOF.
    The configuration of the basis is represented by [X, Y, cos, sin], with the additionnal constraint
    that (cos, sin) should have norm 1. Hence take care when sampling and integrating configurations.
    The robot is depicted as an orange box with two black cylinders (wheels) atop of which is the
    classical (realistic) UR5 model.
    The model also features to OPERATIONAL frames, named "mobile" (on the front of the mobile basis, 30cm
    above the ground) and "tool" (at the very end of the effector).
    '''

    def __init__(self, urdf, pkgs):
        super(MobileRobotWrapper, self).__init__(self, urdf, pkgs, pin.JointModelPlanar())

        M0 = pin.SE3(eye(3), np.matrix([.0, .0, .6]).T)
        self.model.jointPlacements[2] = M0 * self.model.jointPlacements[2]
        self.visual_model.geometryObjects[0].placement = M0 * self.visual_model.geometryObjects[0].placement
        self.visual_data.oMg[0] = M0 * self.visual_data.oMg[0]

        # Placement of the "mobile" frame wrt basis center.
        basisMop = pin.SE3(eye(3), np.matrix([.3, .0, .1]).T)
        self.model.addFrame(pin.Frame('mobile', 1, 1, basisMop, pin.FrameType.OP_FRAME))

        # Placement of the tool frame wrt end effector frame (located at the center of the wrist)
        effMop = pin.SE3(eye(3), np.matrix([.0, .08, .095]).T)
        self.model.addFrame(pin.Frame('tool', 6, 6, effMop, pin.FrameType.OP_FRAME))

        # Create data again after setting frames
        self.data = self.model.createData()

    def initDisplay(self, loadModel):
        RobotWrapper.initDisplay(self, loadModel=loadModel)
        if loadModel and not hasattr(self, 'display'):
            RobotWrapper.initDisplay(self, loadModel=False)

        try:
            # self.viewer.gui.deleteNode('world/mobilebasis', True)
            self.viewer.gui.addBox('world/mobilebasis', .25, .25, .25, [.8, .2, .2, 1])
            self.viewer.gui.addCylinder('world/mobilewheel1', .05, .45, [.1, .0, .0, 1])
            self.viewer.gui.addCylinder('world/mobilewheel2', .05, .45, [.1, .0, .0, 1])

            self.viewer.gui.setStaticTransform('world/mobilebasis', [.0, .0, .35, 1.0, .0, .0, .0])
            self.viewer.gui.setStaticTransform('world/mobilewheel1', [+0.15, .0, .05, .7, .7, .0, .0])
            self.viewer.gui.setStaticTransform('world/mobilewheel2', [-0.15, .0, .05, .7, .7, .0, .0])

            self.viewer.gui.addXYZaxis('world/framebasis', [1., 0., 0., 1.], .03, .1)
            self.viewer.gui.addXYZaxis('world/frametool', [1., 0., 0., 1.], .03, .1)
        except:
            print("Error when adding 3d objects in the viewer ... ignore")

    def display(self, q):
        RobotWrapper.display(self, q)
        M1 = self.data.oMi[1]
        self.viewer.gui.applyConfiguration('world/mobilebasis', pin.se3ToXYZQUATtuple(M1))
        self.viewer.gui.applyConfiguration('world/mobilewheel1', pin.se3ToXYZQUATtuple(M1))
        self.viewer.gui.applyConfiguration('world/mobilewheel2', pin.se3ToXYZQUATtuple(M1))
        self.viewer.gui.refresh()

        pin.framesKinematics(self.model, self.data)
        self.viewer.gui.applyConfiguration('world/framebasis', pin.se3ToXYZQUATtuple(self.data.oMf[-2]))
        self.viewer.gui.applyConfiguration('world/frametool', pin.se3ToXYZQUATtuple(self.data.oMf[-1]))

        self.viewer.gui.refresh()
