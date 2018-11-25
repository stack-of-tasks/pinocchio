//
// Copyright (c) 2016,2018 CNRS
//

#ifndef __pinocchio_frame_hpp__
#define __pinocchio_frame_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include <string>

namespace pinocchio
{
  ///
  /// \brief Enum on the possible types of frame
  ///
  enum FrameType
  {
    OP_FRAME     = 0x1 << 0, // operational frame type
    JOINT        = 0x1 << 1, // joint frame type
    FIXED_JOINT  = 0x1 << 2, // fixed joint frame type
    BODY         = 0x1 << 3, // body frame type
    SENSOR       = 0x1 << 4  // sensor frame type
  };
  
  ///
  /// \brief A Plucker coordinate frame attached to a parent joint inside a kinematic tree
  ///
  template<typename _Scalar, int _Options>
  struct FrameTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pinocchio::JointIndex JointIndex;
    enum { Options = _Options };
    typedef _Scalar Scalar;
    typedef SE3Tpl<Scalar,Options> SE3;
    
    ///
    /// \brief Default constructor of a frame.
    ///
    FrameTpl() : name(), parent(), placement(), type() {} // needed by std::vector
    
    ///
    /// \brief Builds a frame defined by its name, its joint parent id, its placement and its type.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the parent joint in the kinematic tree.
    /// \param[in] previousFrame Index of the parent frame in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    /// \param[in] type The type of the frame, see the enum FrameType
    ///
    FrameTpl(const std::string & name,
             const JointIndex parent,
             const FrameIndex previousFrame,
             const SE3 & frame_placement,
             const FrameType type)
    : name(name)
    , parent(parent)
    , previousFrame(previousFrame)
    , placement(frame_placement)
    , type(type)
    {}
    
    ///
    /// \returns true if *this and other matches and have the same parent, name and type.
    ///
    /// \param[in] other The frame to which the current frame is compared.
    ///
    template<typename S2, int O2>
    bool operator == (const FrameTpl<S2,O2> & other) const
    {
      return name == other.name
      && parent == other.parent
      && previousFrame == other.previousFrame
      && placement == other.placement
      && type == other.type ;
    }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    FrameTpl<NewScalar,Options> cast() const
    {
      typedef FrameTpl<NewScalar,Options> ReturnType;
      ReturnType res(name,
                     parent,
                     previousFrame,
                     placement.template cast<NewScalar>(),
                     type);
      return res;
    }
    
    // data
    
    /// \brief Name of the frame.
    std::string name;
    
    /// \brief Index of the parent joint.
    JointIndex parent;
    
    /// \brief Index of the previous frame.
    FrameIndex previousFrame;
    
    /// \brief Placement of the frame wrt the parent joint.
    SE3 placement;

    /// \brief Type of the frame
    FrameType type;

  }; // struct FrameTpl

  template<typename Scalar, int Options>
  inline std::ostream & operator << (std::ostream& os, const FrameTpl<Scalar,Options> & f)
  {
    os
    << "Frame name: "
    << f.name
    << " paired to (parent joint/ previous frame)"
    << "(" <<f.parent << "/" << f.previousFrame << ")"
    << std::endl
    << "with relative placement wrt parent joint:\n" <<
    f.placement
    << std::endl;
    
    return os;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_frame_hpp__
