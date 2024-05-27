//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_model_item_hpp__
#define __pinocchio_multibody_model_item_hpp__

#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{
  template<typename Derived>
  struct ModelItem : NumericalBase<Derived>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename traits<Derived>::Scalar Scalar;
    enum
    {
      Options = traits<Derived>::Options
    };
    typedef SE3Tpl<Scalar, Options> SE3;

    /// \brief Name of the kinematic element
    std::string name;

    /// \brief Index of the parent joint
    JointIndex parentJoint;

    /// \brief Index of the parent frame
    ///
    /// Parent frame may be unset (to the std::numeric_limits<FrameIndex>::max() value) as it is
    /// mostly used as a documentation of the tree, or in third-party libraries. The URDF parser of
    /// Pinocchio is setting it to the proper value according to the urdf link-joint tree. In
    /// particular, anchor joints of URDF would cause parent frame to be different to joint frame.
    FrameIndex parentFrame;

    /// \brief Position of kinematic element in parent joint frame
    SE3 placement;

    ///
    /// \brief Default constructor of ModelItem.
    ///
    ModelItem()
    : name()
    , parentJoint()
    , parentFrame()
    , placement()
    {
    }

    ///
    /// \brief Builds a kinematic element defined by its name, its joint parent id, its parent frame
    /// id and its placement.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent_joint Index of the parent joint in the kinematic tree.
    /// \param[in] parent_frame Index of the parent frame in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    ///
    ModelItem(
      const std::string & name,
      const JointIndex parent_joint,
      const FrameIndex parent_frame,
      const SE3 & frame_placement)
    : name(name)
    , parentJoint(parent_joint)
    , parentFrame(parent_frame)
    , placement(frame_placement)
    {
    }

    bool operator==(const ModelItem & other) const
    {
      return name == other.name && parentJoint == other.parentJoint
             && parentFrame == other.parentFrame && placement == other.placement;
    }
  };
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_item_hpp__
