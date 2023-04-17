//
// Copyright (c) 2016-2021 CNRS INRIA
//

#ifndef __pinocchio_frame_hpp__
#define __pinocchio_frame_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include <string>

namespace pinocchio
{
  ///
  /// \brief Enum on the possible types of frames.
  ///
  /// \note 
  /// In Pinocchio, the FIXED joints are not included in the kinematic tree but we keep track of them via the vector of frames contained in ModelTpl.
  /// The JOINT frames are duplicate information with respect to the joint information contained in ModelTpl.
  /// 
  /// All other frame types are defined for user convenience and code
  /// readability, to also keep track of the information usually stored within URDF models.
  ///
  /// See also https://wiki.ros.org/urdf/XML/joint, https://wiki.ros.org/urdf/XML/link and https://wiki.ros.org/urdf/XML/sensor.
  ///
  enum FrameType
  {
    OP_FRAME     = 0x1 << 0, ///< operational frame: user-defined frames that are defined at runtime
    JOINT        = 0x1 << 1, ///< joint frame: attached to the child body of a joint (a.k.a. child frame)
    FIXED_JOINT  = 0x1 << 2, ///< fixed joint frame: joint frame but for a fixed joint
    BODY         = 0x1 << 3, ///< body frame: attached to the collision, inertial or visual properties of a link
    SENSOR       = 0x1 << 4  ///< sensor frame: defined in a sensor element
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
    typedef InertiaTpl<Scalar,Options> Inertia;
    
    ///
    /// \brief Default constructor of a frame.
    ///
    FrameTpl()
    : name()
    , parent()
    , placement()
    , type()
    , inertia(Inertia::Zero())
    {} // needed by std::vector
    
    ///
    /// \brief Builds a frame defined by its name, its joint parent id, its placement and its type.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the parent joint in the kinematic tree.
    /// \param[in] previousFrame Index of the parent frame in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    /// \param[in] type The type of the frame, see the enum FrameType.
    /// \param[in] inertia Inertia info attached to the frame.
    ///
    FrameTpl(const std::string & name,
             const JointIndex parent,
             const FrameIndex previousFrame,
             const SE3 & frame_placement,
             const FrameType type,
             const Inertia & inertia = Inertia::Zero())
    : name(name)
    , parent(parent)
    , previousFrame(previousFrame)
    , placement(frame_placement)
    , type(type)
    , inertia(inertia)
    {}
    
    ///
    /// \brief Equality comparison operator.
    ///
    /// \returns true if *this is equal to other.
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
      && type == other.type
      && inertia == other.inertia;
    }

    ///
    /// \returns true if *this is NOT equal to other.
    ///
    template<typename S2, int O2>
    bool operator != (const FrameTpl<S2,O2> & other) const
    {
      return !(*this == other);
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
                     type,
                     inertia.template cast<NewScalar>());
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

    /// \brief Type of the frame.
    FrameType type;
    
    /// \brief Inertia information attached to the frame.
    ///        This inertia will be appended to the inertia supported by the parent joint when calling ModelTpl::addFrame.
    ///        It won't be processed otherwise by the algorithms.
    Inertia inertia;

  }; // struct FrameTpl

  template<typename Scalar, int Options>
  inline std::ostream & operator << (std::ostream& os,
                                     const FrameTpl<Scalar,Options> & f)
  {
    os
    << "Frame name: "
    << f.name
    << " paired to (parent joint/ previous frame)"
    << "(" << f.parent << "/" << f.previousFrame << ")"
    << std::endl
    << "with relative placement wrt parent joint:\n"
    << f.placement
    << "containing inertia:\n"
    << f.inertia
    << std::endl;
    
    return os;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_frame_hpp__
