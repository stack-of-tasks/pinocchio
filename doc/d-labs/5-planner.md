# 5) look ahead (aka motion planning)

## Objective

The objective of this work is to introduce some key algorithm of motion
planning: collision checking, probabilistic roadmaps, visibility PRM,
\f$A^*\f$ (a-star), random shortcut.

## 5.0) prerequisites

### Prerequisite 1
A robot model with simple (static actuated) dynamics,
like a UR5 manipulator robot. See Lab 1 for loading the UR5 robot.

### Prerequisite 2
A collision checking library, provided by Pinocchio.

Pinocchio provides collision checking for a large class of 3D objects
(sphere, capsule, box, triangle soup) using library FCL. Any 3D object
of FCL can be loaded using the C++ interface or the URDF model.
Although, only capsules can yet be added through the Python API, and
only URDF-loaded meshes and Python-loaded capsules can be easily
connected with Gepetto viewer (help is welcome to correct this, it would
requires 4h work).

An example of loading a capsule in Pinocchio and Gepetto viewer is
below:
```py
obs = se3.GeometryObject.CreateCapsule(rad, length)  # Pinocchio obstacle object
obs.name = "obs"                                     # Set object name
obs.parentJoint = 0                                  # Set object parent = 0 = universe
obs.placement = se3.SE3(rotate('x', .1) * rotate('y', .1) * rotate('z', .1), np.matrix([.1, .1, .1]).T)  # Set object placement wrt parent
robot.collision_model.addGeometryObject(obs, robot.model, False) # Add object to collision model
robot.visual_model.addGeometryObject(obs, robot.model, False)    # Add object to visual model
# Also create a geometric object in gepetto viewer, with according name.
robot.viewer.gui.addCapsule("world/pinocchio/" + obs.name, rad, length, [1.0, 0.2, 0.2, 1.0])
```

URDF specifications does not allow to define which collision pairs
should be tested. By default, Pinocchio does not load any collision
pair. A simple strategy is to add all pairs, but often, some meshes of
the models induce wrong collision. Then manually remove them by testing
valid configurations. To be clean, you can store the valid collision
pair in a SRDF file. For UR5:

```py
robot.collision_model.addAllCollisionPairs()
for idx in [56, 35, 23]:
    robot.collision_model.removeCollisionPair(robot.collision_model.collisionPairs[idx])
```

Collision checking are done through the following algorithms:

```py
se3.updateGeometryPlacements(robot.model, robot.data, robot.collision_model, robot.collision_data, q)
se3.computeCollision(robot.collision_model, robot.collision_data, pairId)
se3.computeCollisions(robot.collision_model, robot.collision_data, False)
# last arg to stop early.
```

Both collision algorithms requires a
preliminary update of placement and return `True` if configuration is in
collision (`False` otherwise).

## 5.1) Testing collision

We need to define a simple function to check whether a configuration is
respecting the robot constraints (joint limits and collision, plus any
other inequality-defined constraints you might want).

#### Question 1

Implement the function `check` taking a configuration `q` in
argument and return `True` if and only if `q` is acceptable -- The solution
only uses the 2 collision algorithms of Pinocchio listed above and
standard python otherwise.

## 5.2) Steering method

We need to define a local controller, aka a steering method, to define
the behavior of the robot when it tries to connect to configuration
together. Here we will simply use linear interpolation. More complex
controllers (like optimal trajectories) might be preferred if more
knowledge about the system is available.

In the meantime, we will also need a method to check whether a local
path is collision free. We will do that be simply regularly sampling the
path with a given step length. Continuous collision detection might be
implemented for better and safer performances.

Here we propose to implement the steering method and the path validation
in a single connected method. More versatile implementation is obtained
by defining two different functions.

#### Question 2
Implement a `connect` function, that takes as argument an
initial `q1` and a final `q2` configuration, and return `True` is it is
possible to connect both using linear interpolation while avoiding
collision. Optionally, the function should also returns the sampling of
the path as a list of intermediate configurations -- The solution does
not need any new Pinocchio calls.

## 5.3) Nearest neighbors

Finally, we need a k-nearest-neighbors algorithms.

#### Question 3

Implement a function `nearest_neighbors` that takes as
argument a new configuration `q`, a list of candidates `qs`, and the number
of requested neighbors `k`, and returns the list of the `k` nearest
neighbors of `q` in `qs`. Optionally, the distance function that scores how
close a configuration `q1` is close to a configuration `q2` might be also
provided. If `qs` contains less that `k` elements, simply returns them all
-- no new Pinocchio method is needed for the solution.

## 5.2) Probabilistic roadmap

Basically, probabilistic roadmaps are build by maintaining a graph of
configurations. At each iteration, a new configuration is randomly
sampled and connected to the nearest configurations already in the
graph. The algorithm stops when both start configuration qstart and goal
configuration qgoal can be directly connected to some elements of the
graph.

We propose here to implement the visibility PRM algorithm. This
algorithm also maintains the connected components of the graph. When a
new configuration qnew is sampled, we try to connect it to its nearest
neighbor in each of the already-sampled connected component.
Configuration qnew is added to the graph only if one of the two
following conditions is respected:

- if qnew cannot be connected to any existing connected component
- or if it can be connected to at least two connected component.

In the second case, the connected components that can be connected are
also merged.

A graph structure with connected components is
[provided here](graph_8py_source.html).

#### Question 4
Implement a `visibilityPRM` that takes in argument two
start and goal configurations `qstart` and `qgoal`, and the number of random
sampling that must be achieved before failing. The returns `True` if `qgoal`
can be connected to `qstart`. The graph must also be returned -- no fancy
Pinocchio algorithm is needed here.

The PRM can be visualized in Gepetto-viewer using the function
`display_prm` [provided here](prm__display_8py_source.html).

## 5.4) Searching a path in the roadmap

\f$A^*\f$ is an algorithm to find the shortest path in a graph (discrete
problem). \f$A^*\f$ iterativelly explore the nodes of the graph starting for
the given start. One a node gets explored, its exact cost from start
(cost to here) is exactly known. Then, all its children are added to the
"frontier" of the set of already-explored nodes. Cost from nodes of the
frontier to the goal is not (yet) exactly known but is anticipated
through a heuristic function (also provided). At next iteration, the
algorithm examines a node of the frontier, looking first at the node
that is the most likely to be in the shortest path using the heuristic
distance.

See the fairly complete description of \f$A^*\f$
[provided here](http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html#the-a-star-algorithm).

#### Question 5
Implement the \f$A^*\f$ algorithm. The \f$A^*\f$ returns a sequence of
node ID from start to goal. We only work here with the existing graph of
configuration, meaning that no new nodes a sampled, and no new collision
test are computed. Pinocchio is not necessary here. \f$A^*\f$ returns a list
containing the indexes of the nodes of the PRM graph that one should
cross to reach qgoal from qstart.

## 5.4) Shortcut

Being given a list of configurations, a random shortcut simply randomly
tries to shortcut some subpart of the list: it randomly selects the
start and end of the subpart, and tries to directly connect them (using
the above-defined steering method). In case of success, it skips the
unnecessary subpart. The algorithm iterates a given number of time.

The shortcut can be run on either the list of configuration output by
the \f$A^*\f$, or on a sampling of the trajectory connecting the nodes of the
\f$A^*\f$. We propose here the second version.

#### Question 6
Defines a function `sample_path` to uniformly sample that
trajectory connecting the nodes selected by \f$A^*\f$: for each edge of the \f$A^*\f$
optimal sequence, call `connect` and add the resulting sampling to the
previously-computed sample. It takes as argument the PRM graph and the
list of indexes computed by \f$A^*\f$ and returns a list of robot
configuration starting by qstart and ending by qgoal -- no Pinocchio
method is needed here.

The sampled path can be displayed in Gepetto-viewer using the function
`displayPath` [provided here](prm__display_8py_source.html).

#### Question 7
Implement the `shortcut` algorithm that tries to randomly
connect two configuration of the sampled path. It takes the list of
configuration output by `sample_path` and the number of random shortcut
it should tries. It returns an optimized list of configurations -- no
Pinocchio method is needed here.
