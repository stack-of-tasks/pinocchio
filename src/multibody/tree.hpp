//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_multibody_tree_hpp__
#define __pinocchio_multibody_tree_hpp__

#include "pinocchio/multibody/fwd.hpp"


namespace pinocchio
{
  template<typename Derived>
  struct ModelItem : NumericalBase<Derived>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename traits<Derived>::Scalar Scalar;
    enum { Options = traits<Derived>::Options };
    typedef SE3Tpl<Scalar,Options> SE3;

    /// \brief Name of the geometry object
    std::string name;
    
    /// \brief Index of the parent frame
    ///
    /// Parent frame may be unset (to the std::numeric_limits<FrameIndex>::max() value) as it is mostly used as a documentation of the tree, or in third-party libraries.
    /// The URDF parser of Pinocchio is setting it to the proper value according to the urdf link-joint tree.
    /// In particular, anchor joints of URDF would cause parent frame to be different to joint frame.
    FrameIndex parentFrame;
        
    /// \brief Index of the parent joint
    JointIndex parentJoint;
    
    /// \brief Position of geometry object in parent joint frame
    SE3 placement;

    ///
    /// \brief Default constructor of ModelItem.
    ///
    ModelItem()
    : name()
    , parentFrame()
    , parentJoint()
    , placement()
    {}

    
  };
} //namespace pinocchio

#endif // ifndef __pinocchio_multibody_tree_hpp__
