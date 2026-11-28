//
// Copyright (c) 2017-2024 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody/fwd.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody/fwd.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  template<typename _Scalar, int _Options>
  struct JointCollectionDefaultTpl;
  template<
    typename _Scalar,
    int _Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  template<
    typename _Scalar,
    int _Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;
  template<typename Scalar, int Options = context::Options>
  struct FrameTpl;

  template<int _Dim, typename _Scalar, int _Options = context::Options, int _MaxDim = -1>
  struct JointMotionSubspaceTpl;

  typedef JointMotionSubspaceTpl<1, context::Scalar, context::Options> JointMotionSubspace1d;
  typedef JointMotionSubspaceTpl<3, context::Scalar, context::Options> JointMotionSubspace3d;
  typedef JointMotionSubspaceTpl<6, context::Scalar, context::Options> JointMotionSubspace6d;
  typedef JointMotionSubspaceTpl<Eigen::Dynamic, context::Scalar, context::Options>
    JointMotionSubspaceXd;

  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  typedef Index PairIndex;

  typedef FrameTpl<context::Scalar, context::Options> Frame;
  typedef ModelTpl<context::Scalar, context::Options> Model;
  typedef DataTpl<context::Scalar, context::Options> Data;

  ///
  /// \brief Various conventions to express the velocity of a moving frame
  ///
  /// The velocity of a moving frame is uniquely defined by the velocity of a
  /// point \f$P\f$ of the frame and the angular velocity of the frame. Several
  /// conventions exist in pinocchio depending on the point \f$P\f$ we choose
  /// and on the basis in which the above velocities are projected.
  enum ReferenceFrame
  {
    WORLD = 0, ///<  \f$P\f$ is the point coinciding with the origin of the world frame and the
               ///<  velocities are projected in the basis of the world frame.
    LOCAL = 1, ///<  \f$P\f$ is the origin of the moving frame and the velocities are projected in
               ///<  the basis of the moving frame.
    LOCAL_WORLD_ALIGNED = 2 ///< \f$P\f$ is the origin of the moving frame and the velocities are
                            ///< projected in the basis of the world frame.
  };

  template<ReferenceFrame val>
  struct ReferenceFrameTag
  {
  };

  using WorldFrameTag = ReferenceFrameTag<WORLD>;
  using LocalFrameTag = ReferenceFrameTag<LOCAL>;
  using LocalWorldAlignedFrameTag = ReferenceFrameTag<LOCAL_WORLD_ALIGNED>;

  ///
  /// \brief List of Kinematics Level supported by Pinocchio.
  ///
  enum KinematicLevel
  {
    POSITION = 0,    ///<  Refers to the quantities related to the 0-order kinematics (joint
                     ///<  placements, center of mass position, etc.).
    VELOCITY = 1,    ///<  Refers to the quantities related to the 1st-order kinematics (joint
                     ///<  velocities, center of mass velocity, etc.).
    ACCELERATION = 2 ///<  Refers to the quantities related to the 2nd-order kinematics (joint
                     ///<  accelerations, center of mass acceleration, etc.).
  };

  ///
  /// \brief List of Data allocation strategy.
  ///
  enum struct DataAllocationOption
  {
    ALL = 0,       ///< Allocate all members, including second-order-derivative tensors
    NO_TENSORS = 1 ///< Do not allocate Tensor members
  };

  ///
  /// \brief List of convention to call algorithms.
  ///
  /// The convention will select in witch frame different quantities will be computed.
  enum struct Convention
  {
    ///  Quantities will be computed in world frame (e.g. DataTpl::ov will be filled
    ///  instead of DataTpl::v).
    WORLD = 0,
    ///  Quantities will be computed in local frame (e.g. DataTpl::v will be filled
    ///  instead of DataTpl::ov).
    LOCAL = 1,
  };

  // Forward declaration needed for Model::check
  template<class D>
  struct AlgorithmCheckerBase;

} // namespace pinocchio
