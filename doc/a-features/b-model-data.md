\page md_doc_a-features_b-model-data Model and data

A fundamental paradigm of Pinocchio is the strict separation between
*model* and *data*. By *model*, we mean the physical description of the
robot, including kinematic and possibly inertial parameters defining its
structure. This information is held by a dedicated class which, once
created, is never modified by the algorithms of Pinocchio. By *data*, we
mean all values which are the result of a computation. *Data* vary
according to the joint configuration, velocity, etc\... of the system.
It contains for instance the velocity and the acceleration of each link.
It also stores intermediate computations and final results of the
algorithms in order to prevent memory allocation. With this splitting,
all the algorithms in Pinocchio follow the signature:

```
algorithm(model, data, arg1, arg2, ...)
```

where `arg1, arg2, ...` are the arguments of the function (e.g. configuration or
velocity). Keeping model and data separated reduces memory footprint
when performing several different tasks on the same robot, notably when
this involves parallel computation. Each process can employ its own data
object, while sharing the same model object. The fact that a model
object never changes within an algorithm of Pinocchio enhances the
predictability of the code.

A model can be created using the C++ API or loaded from an external
file, which can be either URDF, Lua (following the RBDL standard) or
Python.
