from math import pi

import numpy as np
from numpy.linalg import norm, pinv

import pinocchio as pin
from pinocchio.utils import cross, zero, rotate, eye
from display import Display


class Visual(object):
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).

    The visual are supposed mostly to be capsules. In that case, the object also contains
    radius and length of the capsule.
    The collision checking computes collision test, distance, and witness points.
    Using the supporting robot, the collision Jacobian returns a 1xN matrix corresponding
    to the normal direction.
    '''
    def __init__(self, name, jointParent, placement, radius=.1, length=None):
        '''Length and radius are used in case of capsule objects'''
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
        if length is not None:
            self.length = length
            self.radius = radius

    def place(self, display, oMjoint):
        oMbody = oMjoint * self.placement
        display.place(self.name, oMbody, False)

    def isCapsule(self):
        return hasattr(self, 'length') and hasattr(self, 'radius')

    def collision(self, c2, data=None, oMj1=None, oMj2=None):
        if data is not None:
            oMj1 = data.oMi[self.jointParent]
            oMj2 = data.oMi[c2.jointParent]
        M1 = oMj1 * self.placement
        M2 = oMj2 * c2.placement

        assert(self.isCapsule() and c2.isCapsule())
        l1 = self.length
        r1 = self.radius
        l2 = c2.length
        r2 = c2.radius

        a1 = M1.act(np.matrix([0, 0, -l1 / 2]).T)
        b1 = M2.act(np.matrix([0, 0, -l2 / 2]).T)
        a2 = M1.act(np.matrix([0, 0, +l1 / 2]).T)
        b2 = M2.act(np.matrix([0, 0, +l2 / 2]).T)

        ab = pinv(np.hstack([a1 - a2, b2 - b1])) * (b2 - a2)

        if (0 <= ab).all() and (ab <= 1).all():
            asat = bsat = False
            pa = a2 + ab[0, 0] * (a1 - a2)
            pb = b2 + ab[1, 0] * (b1 - b2)
        else:
            asat = bsat = True
            i = np.argmin(np.vstack([ab, 1 - ab]))

            pa = a2 if i == 0 else a1
            pb = b2 if i == 1 else b1
            if i == 0 or i == 2:  # fix a to pa, search b
                b = (pinv(b1 - b2) * (pa - b2))[0, 0]
                if b < 0:
                    pb = b2
                elif b > 1:
                    pb = b1
                else:
                    pb = b2 + b * (b1 - b2)
                    bsat = False
            else:  # fix b
                a = (pinv(a1 - a2) * (pb - a2))[0, 0]
                if a < 0:
                    pa = a2
                elif a > 1:
                    pa = a1
                else:
                    pa = a2 + a * (a1 - a2)
                    asat = False

        dist = norm(pa - pb) - (r1 + r2)
        if norm(pa - pb) > 1e-3:
            # Compute witness points
            ab = pa - pb
            ab /= norm(ab)
            wa = pa - ab * r1
            wb = pb + ab * r2

            # Compute normal matrix
            x = np.matrix([1., 0, 0]).T
            r1 = cross(ab, x)
            if norm(r1) < 1e-2:
                x = np.matrix([0, 1., 0]).T
            r1 = cross(ab, x)
            r1 /= norm(r1)
            r2 = cross(ab, r1)
            R = np.hstack([r1, r2, ab])

            self.dist = dist
            c2.dist = dist
            self.w = wa
            c2.w = wb
            self.R = R
            c2.R = R

        return dist

    def jacobian(self, c2, robot, q):
        Ja = pin.jacobian(robot.model, robot.data, q, self.jointParent, False, True)
        Jb = pin.jacobian(robot.model, robot.data, q, c2.jointParent, False, True)

        Xa = pin.SE3(self.R, self.w).action
        Xb = pin.SE3(c2.R, c2.w).action

        J = (Xa * Ja)[2, :] - (Xb * Jb)[2, :]
        return J

    def displayCollision(self, viewer, name='world/wa'):
        viewer.viewer.gui.setVisibility(name, 'ON')
        viewer.place(name, pin.SE3(self.R, self.w))


class Robot(object):
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3).
    The configuration is nq=7. The velocity is the same.
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).

    CollisionPairs is a list of visual indexes.
    Reference to the collision pair is used in the collision test and jacobian of the collision
    (which are simply proxy method to methods of the visual class).
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = pin.Model()
        self.createHand()
        self.data = self.model.createData()
        self.q0 = zero(self.model.nq)
        # self.q0[3] = 1.0
        self.v0 = zero(self.model.nv)
        self.collisionPairs = []

    def createHand(self, root_id=0, prefix='', joint_placement=None):
        def trans(x, y, z):
            return pin.SE3(eye(3), np.matrix([x, y, z]).T)

        def inertia(m, c):
            return pin.Inertia(m, np.matrix(c, np.double).T, eye(3) * m ** 2)

        def joint_name(body):
            return prefix + body + '_joint'

        def body_name(body):
            return 'world/' + prefix + body

        color = [red, green, blue, transparency] = [1, 1, 0.78, 1.0]
        joint_id = root_id
        cm = 1e-2

        joint_placement = joint_placement if joint_placement is not None else pin.SE3.Identity()
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('wrist'))
        self.model.appendBodyToJoint(joint_id, inertia(3, [0, 0, 0]), pin.SE3.Identity())

        L, W, H = 3 * cm, 5 * cm, 1 * cm
        self.viewer.viewer.gui.addSphere(body_name('wrist'), .02, color)
        self.viewer.viewer.gui.addBox(body_name('wpalm'), L / 2, W / 2, H, color)
        self.visuals.append(Visual(body_name('wpalm'), joint_id, trans(L / 2, 0, 0)))

        self.viewer.viewer.gui.addCapsule(body_name('wpalmb'), H, W, color)
        self.visuals.append(Visual(body_name('wpalmb'), joint_id, pin.SE3(rotate('x', pi / 2), zero(3)), H, W))

        self.viewer.viewer.gui.addCapsule(body_name('wpalmt'), H, W, color)
        pos = pin.SE3(rotate('x', pi / 2), np.matrix([L, 0, 0]).T)
        self.visuals.append(Visual(body_name('wpalmt'), joint_id, pos, H, W))

        self.viewer.viewer.gui.addCapsule(body_name('wpalml'), H, L, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([L / 2, -W / 2, 0]).T)
        self.visuals.append(Visual(body_name('wpalml'), joint_id, pos, H, L))

        self.viewer.viewer.gui.addCapsule(body_name('wpalmr'), H, L, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([L / 2, +W / 2, 0]).T)
        self.visuals.append(Visual(body_name('wpalmr'), joint_id, pos, H, L))

        joint_placement = pin.SE3(eye(3), np.matrix([5 * cm, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('palm'))
        self.model.appendBodyToJoint(joint_id, inertia(2, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('palm2'), 1 * cm, W, color)
        self.visuals.append(Visual(body_name('palm2'), joint_id, pin.SE3(rotate('x', pi / 2), zero(3)), H, W))

        FL = 4 * cm
        palmIdx = joint_id

        joint_placement = pin.SE3(eye(3), np.matrix([2 * cm, W / 2, 0]).T)
        joint_id = self.model.addJoint(palmIdx, pin.JointModelRY(), joint_placement, joint_name('finger11'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('finger11'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger11'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger12'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())

        self.viewer.viewer.gui.addCapsule(body_name('finger12'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger12'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL - 2 * H, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger13'))
        self.model.appendBodyToJoint(joint_id, inertia(.3, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addSphere(body_name('finger13'), H, color)
        self.visuals.append(Visual(body_name('finger13'), joint_id, trans(2 * H, 0, 0), H, 0))

        joint_placement = pin.SE3(eye(3), np.matrix([2 * cm, 0, 0]).T)
        joint_id = self.model.addJoint(palmIdx, pin.JointModelRY(), joint_placement, joint_name('finger21'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('finger21'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger21'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger22'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('finger22'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger22'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL - H, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger23'))
        self.model.appendBodyToJoint(joint_id, inertia(.3, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addSphere(body_name('finger23'), H, color)
        self.visuals.append(Visual(body_name('finger23'), joint_id, trans(H, 0, 0), H, 0))

        joint_placement = pin.SE3(eye(3), np.matrix([2 * cm, -W / 2, 0]).T)
        joint_id = self.model.addJoint(palmIdx, pin.JointModelRY(), joint_placement, joint_name('finger31'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('finger31'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger31'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger32'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('finger32'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('y', pi / 2), np.matrix([FL / 2 - H, 0, 0]).T)
        self.visuals.append(Visual(body_name('finger32'), joint_id, pos, H, FL - 2 * H))

        joint_placement = pin.SE3(eye(3), np.matrix([FL - 2 * H, 0, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRY(), joint_placement, joint_name('finger33'))
        self.model.appendBodyToJoint(joint_id, inertia(.3, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addSphere(body_name('finger33'), H, color)
        self.visuals.append(Visual(body_name('finger33'), joint_id, trans(2 * H, 0, 0), H, 0))

        joint_placement = pin.SE3(eye(3), np.matrix([1 * cm, -W / 2 - H * 1.5, 0]).T)
        joint_id = self.model.addJoint(1, pin.JointModelRY(), joint_placement, joint_name('thumb1'))
        self.model.appendBodyToJoint(joint_id, inertia(.5, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('thumb1'), H, 2 * cm, color)
        pos = pin.SE3(rotate('z', pi / 3) * rotate('x', pi / 2), np.matrix([1 * cm, -1 * cm, 0]).T)
        self.visuals.append(Visual(body_name('thumb1'), joint_id, pos, 2 * cm))

        joint_placement = pin.SE3(rotate('z', pi / 3) * rotate('x', pi), np.matrix([3 * cm, -1.8 * cm, 0]).T)
        joint_id = self.model.addJoint(joint_id, pin.JointModelRZ(), joint_placement, joint_name('thumb2'))
        self.model.appendBodyToJoint(joint_id, inertia(.4, [0, 0, 0]), pin.SE3.Identity())
        self.viewer.viewer.gui.addCapsule(body_name('thumb2'), H, FL - 2 * H, color)
        pos = pin.SE3(rotate('x', pi / 3), np.matrix([-0.7 * cm, .8 * cm, -0.5 * cm]).T)
        self.visuals.append(Visual(body_name('thumb2'), joint_id, pos, H, FL - 2 * H))

        # Prepare some patches to represent collision points. Yet unvisible.
        for i in range(10):
            self.viewer.viewer.gui.addCylinder('world/wa%i' % i, .01, .003, [1.0, 0, 0, 1])
            self.viewer.viewer.gui.addCylinder('world/wb%i' % i, .01, .003, [1.0, 0, 0, 1])
            self.viewer.viewer.gui.setVisibility('world/wa%i' % i, 'OFF')
            self.viewer.viewer.gui.setVisibility('world/wb%i' % i, 'OFF')

    def checkCollision(self, pairIndex):
        ia, ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        dist = va.collision(vb, self.data)
        return dist

    def collisionJacobian(self, pairIndex, q):
        ia, ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        return va.jacobian(vb, self, q)

    def displayCollision(self, pairIndex, meshIndex, onlyOne=False):
        ia, ib = self.collisionPairs[pairIndex]
        va = self.visuals[ia]
        vb = self.visuals[ib]
        va.displayCollision(self.viewer, 'world/wa%i' % meshIndex)
        vb.displayCollision(self.viewer, 'world/wb%i' % meshIndex)
        self.viewer.viewer.gui.setVisibility('world/wa%i' % meshIndex, 'ON')
        self.viewer.viewer.gui.setVisibility('world/wb%i' % meshIndex, 'ON')

    def display(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        for visual in self.visuals:
            visual.place(self.viewer, self.data.oMi[visual.jointParent])
        self.viewer.viewer.gui.refresh()
