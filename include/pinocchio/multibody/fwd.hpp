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
  /// \brief List of Reference Frames supported by Pinocchio.
  ///
  enum ReferenceFrame
  {
    WORLD = 0, ///<  The WORLD frame convention corresponds to the frame concident with the Universe/Inertial frame but moving with the moving part (Joint, Frame, etc.).
    LOCAL = 1, ///<  The LOCAL frame convention corresponds to the frame directly attached to the moving part (Joint, Frame, etc.) where the coordinates basis matches the local coordinates system associated with the moving part. It also called the BODY representation in the litterature.
    LOCAL_WORLD_ALIGNED = 2 ///<  The LOCAL_WORLD_ALIGNED frame convention corresponds to the frame centered on the moving part (Joint, Frame, etc.) but with axes aligned with the frame of the Universe. This a MIXED representation betwenn the LOCAL and the WORLD conventions.
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
