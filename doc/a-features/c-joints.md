\page md_doc_a-features_c-joints Joints

Within a model, a robot is represented as a kinematic tree, containing a
collection of all the joints, information about their connectivity, and,
optionally, the inertial quantities associated to each link. In
Pinocchio a joint can have one or several degrees of freedom, and it
belongs to one of the following categories:

- **Revolute** joints, rotating around a fixed axis, either one of \f$X,Y,Z\f$ or a custom one;
- **Prismatic** joints, translating along any fixed axis, as in the revolute case;
- **Spherical** joints, free rotations in the 3D space;
- **Translation** joints, for free translations in the 3D space;
- **Planar** joints, for free movements in the 2D space;
- **Free-floating** joints, for free movements in the 3D space. Planar and free-floating joints are meant to be
  employed as the basis of kinematic tree of mobile robots (humanoids, automated vehicles, or objects in manipulation
  planning).
- More complex joints can be created as a collection of ordinary ones through the concept of **Composite** joint.

Remark: In the URDF format, a joint of type *fixed* can be defined. However,
a **fixed** joint is not really a joint because it cannot move.
For efficiency reasons, it is therefore treated as operational frame of the model.

## From joints to Lie-group geometry

Each type of joints is characterized by its own specific configuration
and tangent spaces. For instance, the configuration and tangent spaces
of a revolute joint are both the real axis line \f$\mathbb{R}\f$, while for
a Spherical joint the configuration space corresponds to the set of
rotation matrices of dimension 3 and its tangent space is the space of
3-dimensional real vectors \f$\mathbb{R}^{3}\f$. Some configuration spaces
might not behave as a vector space, but have to be endowed with the
corresponding integration (exp) and differentiation (log) operators.
Pinocchio implements all these specific integration and differentiation
operators.

See \ref md_doc_a-features_e-lie to go further on this topic.
