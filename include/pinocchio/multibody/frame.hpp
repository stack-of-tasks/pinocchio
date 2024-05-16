//
// Copyright (c) 2016-2021 CNRS INRIA
//

#ifndef __pinocchio_multibody_frame_hpp__
#define __pinocchio_multibody_frame_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model-item.hpp"

#include <string>

namespace pinocchio
{
  ///
  /// \brief Enum on the possible types of frames.
  ///
  /// \note
  /// In Pinocchio, the FIXED joints are not included in the kinematic tree but we keep track of
  /// them via the vector of frames contained in ModelTpl. The JOINT frames are duplicate
  /// information with respect to the joint information contained in ModelTpl.
  ///
  /// All other frame types are defined for user convenience and code
  /// readability, to also keep track of the information usually stored within URDF models.
  ///
  /// See also https://wiki.ros.org/urdf/XML/joint, https://wiki.ros.org/urdf/XML/link and
  /// https://wiki.ros.org/urdf/XML/sensor.
  ///
  enum FrameType
  {
    OP_FRAME = 0x1 << 0, ///< operational frame: user-defined frames that are defined at runtime
    JOINT = 0x1 << 1, ///< joint frame: attached to the child body of a joint (a.k.a. child frame)
    FIXED_JOINT = 0x1 << 2, ///< fixed joint frame: joint frame but for a fixed joint
    BODY =
      0x1 << 3, ///< body frame: attached to the collision, inertial or visual properties of a link
    SENSOR = 0x1 << 4 ///< sensor frame: defined in a sensor element
  };

  template<typename _Scalar, int _Options>
  struct traits<FrameTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
  };

  ///
  /// \brief A Plucker coordinate frame attached to a parent joint inside a kinematic tree
  ///
  template<typename _Scalar, int _Options>
  struct FrameTpl : ModelItem<FrameTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef FrameTpl<_Scalar, _Options> ModelItemDerived;
    typedef typename traits<ModelItemDerived>::Scalar Scalar;
    enum
    {
      Options = traits<ModelItemDerived>::Options
    };
    typedef ModelItem<ModelItemDerived> Base;

    typedef SE3Tpl<Scalar, Options> SE3;
    typedef InertiaTpl<Scalar, Options> Inertia;
    typedef pinocchio::JointIndex JointIndex;

    ///
    /// \brief Default constructor of a frame.
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    FrameTpl()
    : Base()
    , parent(Base::parentJoint)
    , previousFrame(Base::parentFrame)
    , type()
    , inertia(Inertia::Zero())
    {
    } // needed by std::vector
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Builds a frame defined by its name, its joint parent id, its placement and its type.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the parent joint in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    /// \param[in] type The type of the frame, see the enum FrameType.
    /// \param[in] inertia Inertia info attached to the frame.
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    FrameTpl(
      const std::string & name,
      const JointIndex parentJoint,
      const SE3 & frame_placement,
      const FrameType type,
      const Inertia & inertia = Inertia::Zero())
    : Base(name, parentJoint, 0, frame_placement)
    , parent(Base::parentJoint)
    , previousFrame(Base::parentFrame)
    , type(type)
    , inertia(inertia)
    {
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Builds a frame defined by its name, its joint parent id, its placement and its type.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the parent joint in the kinematic tree.
    /// \param[in] parentFrame Index of the parent frame in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    /// \param[in] type The type of the frame, see the enum FrameType.
    /// \param[in] inertia Inertia info attached to the frame.
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    FrameTpl(
      const std::string & name,
      const JointIndex parent_joint,
      const FrameIndex parent_frame,
      const SE3 & frame_placement,
      const FrameType type,
      const Inertia & inertia = Inertia::Zero())
    : Base(name, parent_joint, parent_frame, frame_placement)
    , parent(Base::parentJoint)
    , previousFrame(Base::parentFrame)
    , type(type)
    , inertia(inertia)
    {
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor
    ///
    /// \param[in] other Frame to copy
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    FrameTpl(const FrameTpl & other)
    : Base(other.name, other.parentJoint, other.parentFrame, other.placement)
    , parent(Base::parentJoint)
    , previousFrame(Base::parentFrame)
    , type(other.type)
    , inertia(other.inertia)
    {
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy constructor by casting
    ///
    /// \param[in] other Frame to copy
    ///
    PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
    PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    template<typename S2, int O2>
    explicit FrameTpl(const FrameTpl<S2, O2> & other)
    : Base(
        other.name, other.parentJoint, other.parentFrame, other.placement.template cast<Scalar>())
    , parent(Base::parentJoint)
    , previousFrame(Base::parentFrame)
    , type(other.type)
    , inertia(other.inertia.template cast<Scalar>())
    {
    }
    PINOCCHIO_COMPILER_DIAGNOSTIC_POP

    ///
    /// \brief Copy assignment operator. It needs to be user-define because references cannot be
    /// re-assigned during copy
    ///
    /// \param[in] other Frame to copy
    ///

    FrameTpl<Scalar, Options> & operator=(const FrameTpl<Scalar, Options> & other)
    {
      name = other.name;
      parentJoint = other.parentJoint;
      parentFrame = other.parentFrame;
      placement = other.placement;
      type = other.type;
      inertia = other.inertia;
      return *this;
    }

    ///
    /// \brief Equality comparison operator.
    ///
    /// \returns true if *this is equal to other.
    ///
    /// \param[in] other The frame to which the current frame is compared.
    ///
    template<typename S2, int O2>
    bool operator==(const FrameTpl<S2, O2> & other) const
    {
      return name == other.name && parentJoint == other.parentJoint
             && parentFrame == other.parentFrame && placement == other.placement
             && type == other.type && inertia == other.inertia;
    }

    ///
    /// \returns true if *this is NOT equal to other.
    ///
    template<typename S2, int O2>
    bool operator!=(const FrameTpl<S2, O2> & other) const
    {
      return !(*this == other);
    }

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    FrameTpl<NewScalar, Options> cast() const
    {
      typedef FrameTpl<NewScalar, Options> ReturnType;
      ReturnType res(
        name, parentJoint, parentFrame, placement.template cast<NewScalar>(), type,
        inertia.template cast<NewScalar>());
      return res;
    }

    // data
    /// \brief Index of the parent joint.
    /// \deprecated use \ref parentJoint instead
    PINOCCHIO_DEPRECATED JointIndex & parent;

    /// \brief Index of the previous frame.
    /// \deprecated use \ref parentFrame instead
    PINOCCHIO_DEPRECATED FrameIndex & previousFrame;

    using Base::name;
    using Base::parentFrame;
    using Base::parentJoint;
    using Base::placement;

    /// \brief Type of the frame.
    FrameType type;

    /// \brief Inertia information attached to the frame.
    ///        This inertia will be appended to the inertia supported by the parent joint when
    ///        calling ModelTpl::addFrame. It won't be processed otherwise by the algorithms.
    Inertia inertia;

  }; // struct FrameTpl

  template<typename Scalar, int Options>
  inline std::ostream & operator<<(std::ostream & os, const FrameTpl<Scalar, Options> & f)
  {
    os << "Frame name: " << f.name << " paired to (parent joint/ parent frame)"
       << "(" << f.parentJoint << "/" << f.parentFrame << ")" << std::endl
       << "with relative placement wrt parent joint:\n"
       << f.placement << "containing inertia:\n"
       << f.inertia << std::endl;

    return os;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_frame_hpp__
