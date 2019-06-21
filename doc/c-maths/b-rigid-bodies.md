# Rigid Bodies

## Geometry

A rigid body system is an assembly of different parts which are joints, rigid bodies and forces. A joint connect two differents bodies and gather all kinematics relations between those two bodies. The joints can create movements or follow t but in any case, they are at the centre of the movement. The movement is described by breaking it down into three parts, rotations, translations or both.

In robotic, the rotation matrices described above form a set called, the **Special Orthogonal** group \f$ SO \f$. There are two groups within the latter, the one that gathers the 3 by 3 square matrices is the group \f$ So(3) \f$. It takes account all rotations in the three-dimensionnal space. The second group is useful if you focus essentially on a single rotation, for example for a flat movement. it gathers the square matrices of size 2 by 2, it is the group \f$ SO(2) \f$. 

The set that brings together all the homogeneous transformations matrices is the **Special Euclidean** group \f$ SE \f$. As with the rotation matrices, there are two differents groups, \f$ SE(3) \f$ which brings together movements in physical space and \f$ SE(2) \f$ useful for flat movements. 

### Using quaternions for a \f$ SO(3) \f$ object 

To use quaternions for a \f$ SO(3) \f$ object we have several methods, we can do as in the \f$ SE(3) \f$ example in the **e-lie** chapter by removing the translation vector. 

Or we can just consider one rotation instead of two. For example, in a landmark link to the robot itself, we consider the starting position as the origine of this landmark.  

So let's consider a cube in the 3-dimensional space.
![Rotation of a cube around its diagonal](rotation_resize.gif)

We want to determine the image of a vector \f$ \overrightarrow{v} \f$ by a \f$ 120° \f$ rotation (\f$ \frac{2\pi}{3}) \f$ around the big diagonal of the cube, let's call it \f$ \overrightarrow{r} \f$. We have to use a passage through quaternion. We have\f$ \overrightarrow{v} = \overrightarrow{i} + \overrightarrow{k} = \begin{pmatrix} 1 \\ 0 \\ 1 \end{pmatrix} \f$ and \f$ \overrightarrow{r} = \overrightarrow{i} + \overrightarrow{j} + \overrightarrow{k} = \begin{pmatrix} 1 \\ 1 \\ 1 \end{pmatrix} \f$

We compute the corresponding quaternion :

\f$ q = cos(\alpha/2) + sin(\alpha/2) * \frac{\overrightarrow{r}}{||\overrightarrow{r}||} \f$

Therefore we have :

\f$ q = \frac{1}{2} + \frac{1}{2} * (\overrightarrow{i} + \overrightarrow{j} + \overrightarrow{k}) \f$

And so we can compute the image of vector \f$ \overrightarrow{v} \f$ using :

\f$ \overrightarrow{v'} = q * \overrightarrow{v} *q^{-1} \f$

we have :

\f$ \overrightarrow{v'} = \overrightarrow{i} + \overrightarrow{j} = \begin{pmatrix} 1 \\ 1 \\ 0 \end{pmatrix} \f$


### Benefits of using quaternions

Determining the matrix corresponding to a rotation is not immediate, so that very often the physical problem is defined by the couple \f$ (\alpha,\overrightarrow{r} ) \f$. Another problem related to the composition of rotations is known as "blocking of Cardan" : we can see it in specific cases, for example when two successive joints have close or even aligned axes of rotation. In this case,a very large variation of the first angle does not change the position of the end of the device. A robot could then generate very strong violent movements without realizing it, due to the approximation of the calculations. To remedy these two points, we use quaternions.



### Cartesian product

Of course the cartesian product is essential for analysis and description of the movement in our euclidean space. But here, it's specific to the lie algebra, this is different from the cartesian product which define our space. 
The cartesian product can also be used to create a specific space by associating spaces related to the lie algebra as \f$ SE \f$ and \f$ SO \f$ groups. 

For example let's consider a robot on wheels like Tiago, it just can moved rolling on the ground, it is possible to assimilate the ground as a plan. And the robot can rotated around the z-axis so we have to deal with a \f$ SE(2) \f$ object. Then we add to it a articulated arm with four revolute joints spread out his arm, each has one degree of freedom of rotation so they are \f$ SO(2) \f$ objects. To deal with this set we use the cartesian product related to the lie algebra and we get a new space in which we are able to manage all trajectories the robot and his arm can have.


### Vector space 

If you want to create a tangent space to simplify calculations of a trajectory it is necessary to use vector spaces. Indeed, a tangent space is a vector space that is the whole of all velocity vectors. 
Let's consider an object having a trajectory, all points of it have a velocity which is tangent to the trajectory and the space associate to one velocity and passing by one point of the trajectory is the \b tangent \b space. 


Furthermore, by using vector spaces we have the possibility to use its properties as those of the Euclidean cross operator and linear combinations.
However it is important to know that "vector space" is here related to **Lie algebra** and this is different for a vector space we used to deal with.

## Kinematics

## Dynamics
