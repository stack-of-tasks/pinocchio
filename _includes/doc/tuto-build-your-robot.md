<!-- MarkdownTOC -->

- [Objectives](#objectives)
- [Tutorial 1.0. Tips](#tutorial-1-tips)
    - [Ubuntu](#ubuntu)
    - [Python](#python)
    - [Numpy](#numpy)
    - [Pinocchio](#pinocchio)
    - [SE3 objects](#se3-objects)
- [Tutorial 1.1. Display cubes, cylinder and spheres](#tutorial-11-display-cubes-cylinder-and-spheres)
    - [Create the display](#create-the-display)
    - [Create simple objects](#create-simple-objects)
    - [Moving objects](#moving-objects)
- [Tutorial 1.2. Creating the robot model](#tutorial-2-creating-the-robot-model)
    - [Model and Data](#model-and-data)
    - [Creating a Model](#creating-a-model)
    - [Joint Models](#joint-models)
    - [Associating visual object to bodies](#associating-visual-object-to-bodies)
    - [The robot class](#the-robot-class)
- [Tutorial 1.3. Displaying the model](#tutorial-3-displaying-the-model)
    - [Computing the robot geometry](#computing-the-robot-geometry)
    - [Displaying the model](#displaying-the-model)
    - [Move your robot](#move-your-robot)

<!-- /MarkdownTC -->

<a name="objectives"></a>

# Objectives


The objectives of the first tutorial is to understand the basic principles of the robot-model software Pinocchio: How a model is stored? How the model can be displayed in 3D using the software Gepetto-viewer? How the model computation routines can be called?

Mostly, we focus here on forward geometry.

At the end of the tutorial, you should have a kinematic model of the robot of your dreams, with simple 3D geometry displayed in Gepetto-viewer. All that should be stored in a simple class Robot that would be used throughout the following tutorials.

<a name="tutorial-1-tips"></a>

# Tutorial 1.0. Tips
<a name="ubuntu"></a>

## Ubuntu

The tutorial environment would typically be a Ubuntu 12.04 computer with Pinocchio and Gepetto-viewer installed. VirtualBox images with both software already installed are available on demand.

Once your Ubuntu is started, you can start a new terminal by typing CTRL-ALT-T. The shortcut will typically launch a gnome-terminal process. Inside the gnome-terminal window, you can open new terminals (in new tabulations) by typing CTRL-SHIFT-T, and navigate between tabulations with CTRL-PgUp and CTRL-PgDn.

<a name="python"></a>

## Python

All the tutorials will use the Python language.

You can open a python terminal in your own shell. Simply type :


    student@pinocchio1204x32vbox:~$ ipython
    >>>

Afterwards you will just have to type your commands in this newly opened terminal.

To close the python terminal, just type CTRL-D (possibly typing CTRL-C first to interrupt any on-going execution).

The code that you enter inside the interactive shell is lost. It must only be used for debugging and simple experiments. Rather store the code that you produce inside a Python script file. For example, you can place the script files in /home/student/src/student. Name it with the ID of the tutorial in prefix, and .py in suffix. You can use any editor you want for editing the python script. The most basic one is gedit.

    student@pinocchio1204x32vbox:~$ cd ~/src/student
    student@pinocchio1204x32vbox:~/src/student$ gedit tuto1_q1.py


You can then launch this script before entering in interactive mode:


    student@pinocchio1204x32vbox:~/src/student$ ipython -i tuto1_q1.py
    >>>


You can also run it without entering the interactive mode (ie. without starting a new python terminal). In your shell, just type:

    student@pinocchio1204x32vbox:~/src/student$ ipython tuto1_q1.py
    student@pinocchio1204x32vbox:~/src/student$

<a name="numpy"></a>

## Numpy

NumPy (Numerics for Python) is the standard linear algebra library for Python. It contains two main classes: array and matrices, matrices being 2-dimension arrays with a dedicated algebra. Numpy is later extended with scientific algorithms (SciPy) and graphical plots (MatPlotLib), that we will use as well in following tutorials.

In the following, we will use numpy Matrix class to represent matrices and vectors. In Numpy, vectors simply are matrices with one column. See the following example.

{% highlight python %}
    # Examples of matrix-vector operations with Numpy.

import numpy as np
A = np.matrix([ [1,2,3,4],[5,6,7,8]]) # Define a 2x4 matrix
b = np.zeros([4,1])   # Define a 4 vector (ie a 4x1 matrix) initialized with 0.
c = A*b               # Obtain c by multiplying A by b.
{% endhighlight %}

A bunch of useful functions are packaged in the utils of pinocchio.

{% highlight python %}

    # Example of matrix-vector shortcut functions with Numpy inside Pinocchio.

from pinocchio.utils import *
eye(6)                    # Return a 6x6 identity matrix
zero(6)                   # Return a zero 6x1 vector
zero([6,4])               # Return a zero 6x4 matrix
rand(6)                   # Random 6x1 vector
isapprox(zero(6),rand(6)) # Test epsilon equality
mprint(rand([6,6]))       # Matlab-style print
skew(rand(3))             # Skew "cross-product" 3x3 matrix from a 3x1 vector
cross(rand(3),rand(3))    # Cross product of R^3
rotate('x',0.4)           # Build a rotation matrix of 0.4rad around X.
{% endhighlight %}

A pretty matlab-style print of matrices is implemented by the function mprint.

{% highlight python %}
mprint(eye(3))
{% endhighlight %}


<a name="pinocchio"></a>

## Pinocchio

Pinocchio typically requires the following imports. It is advice to start any of your script by the following few lines.

{% highlight python %}

    # Typical header of a Python script using Pinocchio
from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import gepetto.corbaserver
{% endhighlight %}


<a name="se3-objects"></a>

## SE3 objects

Pinocchio is implementing dedicated mathematics classes to represent SE3 objects. For this tutorial, we will only need rigid displacement SE3. The class is implemented in Pinocchio under the name SE3. It uses the representation by rotation matrix 3x3 and translation vector 3x1. You can access and modify the translation and rotation. The neutral element of the group is SE3.Identity(). A random displacement can be generated using SE3.Random(). Elements of SE3 can be multiplied together. Inverse in SE3 are obtained by the method SE3.inverse. 3D vector can be moved also by multiplying with SE3 objects.

{% highlight python %}

    # Examples of SE3 operations in Pinocchio.
print se3.SE3
I44 = se3.SE3.Identity()
Mrand = se3.SE3.Random()
print(Mrand)
print(Mrand.rotation,Mrand.translation)
Mrand.translation = zero(3)
Mrand.rotation = eye(3)
oM1,oM2 = se3.SE3.Random(),se3.SE3.Random()
_2Mo = oM2.inverse()
_2M1 = _2Mo*oM1
_1p = rand(3)
op = oM1*_1p
{% endhighlight %}


<a name="tutorial-11-display-cubes-cylinder-and-spheres"></a>

# Tutorial 1.1. Display cubes, cylinder and spheres
<a name="create-the-display"></a>

## Create the display

For display, we will use Gepetto-viewer, a simple 3D renderer built on top of OpenSceneGraph (try to google this name). Gepetto-viewer is a standalone Corba server. It means that the display will run permanently as an independent Linux process, without having to be restarted each time your restart your Python script. Your scripts will communicate using the middleware Corba with the display (do not be afraid, for you it will be transparent).

In your script, you need to import the Gepetto-viewer client library, send connect with the server, possibly create 3D objects and then move these objects while your robot is moving.

First, you need to start the server in a first terminal bash terminal …

    student@pinocchio1204x32vbox:~$ gepetto-viewer-server

Your own Python scripts can be started from a second terminal. The following class do all the client work for you. Just copy-paste it.

{% highlight python %}

    # Example of a class Display that connect to Gepetto-viewer and implement a
    # 'place' method to set the position/rotation of a 3D visual object in a scene.
class Display():

    # Class Display: Example of a class implementing a client for the Gepetto-viewer server. The main method of the class is 'place', that sets the position/rotation of a 3D visual object in a scene.

    def __init__(self,windowName = "pinocchio" ):
        '''
        This function connect with the Gepetto-viewer server and open a window with the given name.
        If the window already exists, it is kept in the current state. Otherwise, the newly-created
        window is set up with a scene named 'world'.
        '''

        # Create the client and connect it with the display server.
        try:
            self.viewer=gepetto.corbaserver.Client()
        except:
            print "Error while starting the viewer client. "
            print "Check whether Gepetto-viewer is properly started"

        # Open a window for displaying your model.
        try:
            # If the window already exists, do not do anything.
            windowID = self.viewer.gui.getWindowID (windowName)
            print "Warning: window '"+windowName+"' already created."
            print "The previously created objects will not be destroyed and do not have to be created again."
        except:
            # Otherwise, create the empty window.
            windowID = self.viewer.gui.createWindow (windowName)
            # Start a new "scene" in this window, named "world", with just a floor.
            self.viewer.gui.createSceneWithFloor("world")
            self.viewer.gui.addSceneToWindow("world",windowID)

        # Finally, refresh the layout to obtain your first rendering.
        self.viewer.gui.refresh()

    def nofloor(self):
        '''
        This function will hide the floor.
        '''
        self.viewer.gui.setVisibility('world/floor',"OFF")
        self.viewer.gui.refresh()

    def place(self,objName,M,refresh=True):
        '''
        This function places (ie changes both translation and rotation) of the object
        names "objName" in place given by the SE3 object "M". By default, immediately refresh
        the layout. If multiple objects have to be placed at the same time, do the refresh
        only at the end of the list.
        '''
        self.viewer.gui.applyConfiguration(objName,
                                           XYZQUATToViewerConfiguration(se3ToXYZQUAT(M)) )
        if refresh: self.viewer.gui.refresh()

{% endhighlight %}

You can create an instance of display with no argument. The Display class will likely be used in all your scripts. You can generalize the work by put the class alone in a Python file named display.py. Then import it in any of your script by doing:

{% highlight python %}

    # Example of creation of an object of class Display (implemented in the previous example, inside a file 'display.py').
    #Do not forget to start Gepetto-viewer server in another terminal before creating the client.
from display import Display
display = Display()
{% endhighlight %}

<a name="create-simple-objects"></a>

## Create simple objects

Any object of Gepetto viewer is referenced by a name inside a scene. For example, a box named "box" in scene "world" would be referenced by the path "world/box".

Put all you objects in the general scene "world". You can use any names.

A box of dimension 2x2x2 can be created using the function

{% highlight python %}

    # Example of use of the class Display to create a box visual object.
boxid   = 147
name    = 'box' + str(boxid)
[w,h,d] = [1.0,1.0,1.0]
color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
display.viewer.gui.addBox('world/'+name, w,h,d,color)
{% endhighlight %}



A sphere of dimension 1x1x1 can be created using:

{% highlight python %}

    # Example of use of the class Display to create a sphere visual object.
display.viewer.gui.addSphere('world/sphere', 1.0,color)
{% endhighlight %}


A cylinder of dimension 1x1x1 can be created using:

{% highlight python %}

    # Example of use of the class Display to create a cylinder visual object.
radius = 1.0
height = 1.0
display.viewer.gui.addCylinder('world/cylinder', radius,height,color)
{% endhighlight %}

Other 3D primitives can be created as well. Checkout the gepetto-viewer IDL in bash terminal

```
student@pinocchio1204x32vbox:~$ less /home/student/src/viewer/gepetto-viewer-corba/idl/gepetto/viewer/graphical-interface.idl
```

<a name="moving-objects"></a>

## Moving objects

Place an object by simply mentioning its name and the placement you want. Gepetto-Viewer expects a XYZ-Quaternion(w,x,y,z) representation of the SE3 placement. The translation is performed by the method "place" of your class "Display". After placing an object, the layout must be explicitly refreshed. This is to reduce the display load when flushing several objects at the same time. Typically, during robot movements, you would only ask one refresh per control cycle. The method "Display.place" refreshes the layout by default.

{% highlight python %}

    # Example of use of the class display to place the previously-create object at random SE3 placements.
display.place("world/box147",se3.SE3.Random(),False)
display.place("world/sphere",se3.SE3.Random(),False)
display.place("world/cylinder",se3.SE3.Random())
{% endhighlight %}

<a name="tutorial-2-creating-the-robot-model"></a>

# Tutorial 1.2. Creating the robot model
<a name="model-and-data"></a>

## Model and Data

Pinocchio models are separated in two classes. The class "Model" gathers all the constant elements. The class "Data" is generated from an object "Model". It gathers all the swap memory that is used by the algorithms of Pinocchio to perform the computations. Typically, an algorithm will take a Model and a Data, let Model unchanged and put the results in Data.

The two classes are se3.Model and se3.Data.

<a name="creating-a-model"></a>

## Creating a Model

An empty model is created using the method se3.Model.BuildEmptyModel().

When created, a model contains a single couple joint/body (representing the universe, with a trivial mass and inertia).

Bodies and joints are added by pair in a recursive fashion, using the method Model.addJointAndBody. In pinocchio we handle the kinematic chain as a list of joints (_Model.joints_) with the inertial informations of the supported body:

{% highlight python %}

    # Example of creation of a simple robot model with one single revolute joint rotating around axis X.
model = se3.Model.BuildEmptyModel()
name               = "first"                             # Prefix of the name.
jointName,bodyName = [name+"_joint",name+"_body"]        # Names of joint and body.
jointPlacement     = se3.SE3.Random()                    # SE3 placement of the joint wrt chain init.
parent             = 0                                   # Index of the parent (0 is the universe).
jointModel         = se3.JointModelRX()                  # Type of the joint to be created.
bodyInertia        = se3.Inertia.Random()                # Body mass/center of mass/inertia => useless for
                                                         # this tutorial.

model.addJointAndBody(parent, jointModel, jointPlacement, bodyInertia, jointName, bodyName)
print('Model dimensions: {:d}, {:d}, {:d}'.format(model.nbody,model.nq,model.njoint))
{% endhighlight %}

<a name="joint-models"></a>

## Joint Models

In the previous code, the joint is a revolute that rotates around the X axis. Y and Z axis are also available.

Other joints are available in Pinocchio. The most classical to use are the revolutes and the free-flyers (se3.JointModelFreeFlyer). Free flyer is typically the first joint for mobile robots such as humanoids. More to come in the next tutorial about free flyer joints.

<a name="creating-the-data"></a>

## Creating the data

After all the joint have been added, a data object can be generated using the method Model.createData.

{% highlight python %}

    # Example of how to create a 'Data' object from a 'Model' object.
data = model.createData()
{% endhighlight %}

<a name="associating-visual-object-to-bodies"></a>

## Associating visual object to bodies

For any body, you can create one or several visual primitive objects and "attach" them to a parent joint with an arbitrary SE3 relative placement.
To compute the absolute placement of these visuals you can left-multiply the relative placement by the absolute placement of the parent joint ( think of the Chasles Relation)
For each body of your model, you cant hus define a list of pair (3D name, 3D relative placement). Very likely, each body should have zero or one element in its corresponding list. You can store all these lists in a list named "visuals", containing "model.nbody" elements.

The visual elements of the scene (i.e. static elements that are not attached to the robot) can be attached to the "universe" joint 0 of the model, in the first element of the "visuals" list.

<a name="the-robot-class"></a>

## The robot class

The objective of this tutorial is to gather all the elements explained above to create a single Robot class. The class must contains the 4 following elements:
- A model object
- A data object
- A display object
- A list of visual objects

__Question 1:__ Create a class Robot as specified previously. Store the results in a dedicated python file "robot.py".


<a name="tutorial-3-displaying-the-model"></a>

# Tutorial 1.3. Displaying the model

<a name="computing-the-robot-geometry"></a>

## Computing the robot geometry

The algorithms of Pinocchio are implemented as static functions, that take in two first arguments a robot model and the corresponding data, with typically additional arguments specifying the robot configuration, velocity, etc.

The function "se3.geometry" computes the placement of all the robot joints with respect to the world reference. It takes as input the robot configuration, i.e. a vector "q" of dimension robot.nq. The function has no direct output. The results are stored in the table data.oMi. This table contains model.njoint elements. Each element is a SE3 object specifying the placement of the corresponding joint.

{% highlight python %}

    # Example of use of the function 'geometry' in Pinocchio.
q = rand(model.nq)
se3.geometry(model,data,q)
for i in range(1,model.nbody):
    print model.names[i],": ",data.oMi[i]
{% endhighlight %}

<a name="displaying-the-model"></a>

## Displaying the model

Consider joint "ijoint" to which one visual "ivisual" is attached. The placement of the joint in the world is data.oMi[ijoint]. The name of the visual is `visuals[ijoint][ivisual][0]`. . The placement of the visual in the parent joint frame is `visuals[ijoint][ivisual][1]`.

{% highlight python %}

    # Example of computation of the placement of the visual object.
ibody = 1
ivisual = 0
oMbody = data.oMi[ibody]
visualName =  visuals[ibody][ivisual][0]
bodyMvisual = visuals[ibody][ivisual][1]
oMvisual = oMbody*bodyMvisual
print visualName,": ",oMvisual
{% endhighlight %}

__Question 2:__ Loops across all the visuals to display them on the Gepetto-Viewer layout. Remember to only refresh the layout at the end of the display loop. Place this algorithm in a Robot.display method, that takes the robot configuration as a vector "q", computes the geometry and display the visuals at the correct places.

<a name="move-your-robot"></a>

## Move your robot

__Question 3:__ From an initial position q0, chose an arbitrary velocity v and integrate it with small (1e-3) timestep, while displaying the robot configuration at every integration step.
