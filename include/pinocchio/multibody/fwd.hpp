//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_fwd_hpp__
#define __pinocchio_multibody_fwd_hpp__

#include "pinocchio/fwd.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
  PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  /**
   * \addtogroup pinocchio_multibody
   * @{
   */
  template<typename Scalar, int Options = context::Options>
  struct FrameTpl;
  PINOCCHIO_COMPILER_DIAGNOSTIC_POP

  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  typedef Index PairIndex;

  typedef FrameTpl<context::Scalar, context::Options> Frame;

  typedef ModelTpl<context::Scalar, context::Options> Model;
  typedef DataTpl<context::Scalar, context::Options> Data;

  struct GeometryModel;
  struct GeometryData;

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

  /**
   * @}
   */
  // end of group multibody

  // Forward declaration needed for Model::check
  template<class D>
  struct AlgorithmCheckerBase;

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_fwd_hpp__
