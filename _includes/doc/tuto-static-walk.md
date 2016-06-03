<!-- MarkdownTOC -->

- [Objectives](#objectives)
- [Tutorial 3.0. Prerequisites](#tutorial-30-prerequisites)
- [Tutorial 3.1. Integrate](#tutorial-31-integrate)
- [Tutorial 3.2: Move your hands](#tutorial-32-move-your-hands)
- [Tutorial 3.2: static walk](#tutorial-32-static-walk)
- [Tutorial 3.3: posture](#tutorial-33-posture)

<!-- /MarkdownTC -->


<a name="objectives"></a>

# Objectives

The main objective of this tutorial is to apply an inverse kinematics. As an application background, we will build a static-walk movement.

<a name="tutorial-30-prerequisites"></a>

# Tutorial 3.0. Prerequisites

We will work directly with a humanoid robot. We assume that the free flyer configuration is represented by a position+quaternion joint (JointModelFreeFlyer) and that all the other joints are simple (i.e. vector-space configuration).

For example, the model of Romeo can be used. See the previous tutorial.

<a name="tutorial-31-integrate"></a>

# Tutorial 3.1. Integrate

First we need to integrate a velocity, i.e. knowing a velocity of the robot vq, what is the configuration q1 reached from q0 after dt seconds?

The two functions below can be used to pass from a position+quaternion representation of a placement to a homogeneous matrix.

{% highlight python %}
# Convertion from translation+quaternion to SE3 and reciprocally
q2m = lambda q: se3.SE3( se3.Quaternion(q[6,0],q[3,0],q[4,0],q[5,0]).matrix(), q[:3])
m2q = lambda M: np.vstack([ M.translation,se3.Quaternion(M.rotation).coeffs() ])
{% endhighlight %}

The exponential of SE3 is already coded in Pinocchio.

{% highlight python %}
import pinocchio as se3
nu    = rand(6)
M     = se3.exp(nu)
print   nu - se3.log(M).vector()  # log returns a se3.Motion object.
{% endhighlight %}

To integrate vq starting from q0, first extract the homogeneous placement M from q0 using q2m, compute the displacement dM due to vq using the exponential map, multiply the first by the second (take care, they do not commute), and convert the results back to a vector notation using m2q.

__Question 1:__ Write a function integrate(q,vq) that compute the position reached from q after applying a constant velocity vq during 1 seconds.

<a name="tutorial-32-move-your-hands"></a>

# Tutorial 3.2: Move your hands

Next, let s do an inverse kinematics. We will consider three different kind of tasks: an error in SE3 to reach a reference placement, an error on the center of mass (COM) position in R^3, and an error directly in the configuration space. We start by the difficult one.

The function `se3.jacobian(model,data,q,IDX,True,True)` can be used to compute the Jacobian of body IDX in the frame of joint IDX (the two last arguments force the value to be in the local frame and to be recomputed).

{% highlight python %}
# Compute the jacobian of the right hand in the local frame.
IDX = robot.rh # right hand in Romeo.
J = se3.jacobian(model,data,q,IDX,True,True)
{% endhighlight %}

We already saw in previous tutorials that the placement of body IDX is available in data.oMi[IDX] (remember to call se3.geometry before to refresh oMi).

{% highlight python %}
# Compute the placement of the right hand wrt world frame.
se3.geometry(robot.model,robot.data,q)
Mrh = robot.data.oMi[robot.rh] # Placement of the right hand.
{% endhighlight %}

Now assume that the reference placement to be reached is Mdes, and that the current placement is stored in M. The spatial velocity to go from M to Mdes, expressed in frame M, is log(M.inverse()*Mdes) (try to prove it).

We now have both equation:

. direct kinematics: nu = J*vq

. reference task movemetn: nu_des = log(M*Mdes) (multiply by a gain k=0.1 for slower movement)

Finally, the velocity to be sent to the robot is vq = pinv(J)*nu_des (pinv is available in numpy.linalg)

__Question 2:__ Build a control law to bring the right hand ~15cm in front of the head of the robot. What are the movements of the feet?

Other tasks might be considered, by simply stacking jacobians and task references

{% highlight python %}
J = numpy.vstack([J1,J2,J3])
v = numpy.vstack([v1,v2,v3])
{% endhighlight %}

__Question 3:__ Build a control law to bring the hand to a reference placement while keeping both feet on the ground.

<a name="tutorial-32-static-walk"></a>

# Tutorial 3.2: static walk

The COM can be computed by se3.centerOfMass; its jacobian by se3.jacobianCenterOfMass.

{% highlight python %}
c = se3.centerOfMass(robot.model,robot.data,q)
Jc = se3.jacobianCenterOfMass(robot.model,robot.data,q)
{% endhighlight %}

The COM task is simply the difference between the current COM c and a reference 3D vector cdes: ec = c-cdes. The reference movement is the corresponding exponential decay: vc = -kc*ec, with kc the gain.

This task (ec,Jc,vc) can be added to the previous controller to move the hand and the COM while keeping the feet static.

__Question 4:__ Build a static walk by sequencing four controllers: first move the COM below the right foot while keeping both feet static; then move the left foot up and forward while keeping the other foot and the COM fixed; make the foot land; and finally bring the COM forward between the two feet while they are static.

<a name="tutorial-33-posture"></a>

# Tutorial 3.3: posture

Consider a matrix J. The corresponding null-space projector can be computed by N=I-pinv(J)*J.

Now consider N tasks indexed by i (e[i],J[i],v[i]). The hierarchical control law can be obtained by a recurence over i:

. vq = vq + N*pinv(J[i]*N) * (vi - Ji*vq)

. N = N - N*pinv(J[i]*N)*J[i]*N

starting from vq=zero(nv) and N=eye(nv).

__Question 5:__ Implement a function hqp(Js,vs), where Js is a list of jacobians, and vs a list of reference velocities, that computes implement the hierarchical loop.

The posture task tries to keep the actuated joints to a reference position (typically the initial one). The task is defined by:

. Jq = np.hstack( [ zero([nv-6,7]),eye(nv-6) ])

. eq = q[7:] - qref[7:]

. vq = -kq * eq

__Question 6:__ Implement a static walk with a reference posture.
