//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_fwd_hpp__
#define __pinocchio_multibody_fwd_hpp__

#include "pinocchio/fwd.hpp"

#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  /**
   * \addtogroup pinocchio_multibody
   * @{
   */

  template<typename Scalar, int Options=0> struct FrameTpl;

  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;

  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  typedef Index PairIndex;
  
  typedef FrameTpl<double> Frame;
  
  typedef ModelTpl<double> Model;
  typedef DataTpl<double> Data;
  
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
    WORLD = 0, ///<  \f$P\f$ is the point coinciding with the origin of the world frame and the velocities are projected in the basis of the world frame.
    LOCAL = 1, ///<  \f$P\f$ is the origin of the moving frame and the velocities are projected in the basis of the moving frame.
    LOCAL_WORLD_ALIGNED = 2 ///< \f$P\f$ is the origin of the moving frame and the velocities are projected in the basis of the world frame.
  };

  ///
  /// \brief List of Kinematics Level supported by Pinocchio.
  ///
  enum KinematicLevel
  {
    POSITION = 0, ///<  Refers to the quantities related to the 0-order kinematics (joint placements, center of mass position, etc.).
    VELOCITY = 1, ///<  Refers to the quantities related to the 1st-order kinematics (joint velocities, center of mass velocity, etc.).
    ACCELERATION = 2 ///<  Refers to the quantities related to the 2nd-order kinematics (joint accelerations, center of mass acceleration, etc.).
  };

  /**
   * @}
   */
  // end of group multibody

  // Forward declaration needed for Model::check
  template<class D> struct AlgorithmCheckerBase;

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_fwd_hpp__
