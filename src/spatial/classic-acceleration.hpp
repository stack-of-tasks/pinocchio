//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_spatial_classic_acceleration_hpp__
#define __pinocchio_spatial_classic_acceleration_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"

namespace pinocchio
{
  ///
  /// \brief Computes the classic acceleration from a given spatial velocity and spatial acceleration.
  ///
  /// \tparam Motion1 type of the input spatial velocity.
  /// \tparam Motion2 type of the input spatial acceleration.
  /// \tparam Vector3Like type of the return type (a type similar to a 3D vector).
  ///
  /// \param[in] spatial_velocity input spatial velocity.
  /// \param[in] spatial_acceleration input spatial acceleration.
  /// \param[out] res computed classic acceleration.
  ///
  /// \remarks To be valid, the spatial velocity and the spatial acceleration have to be expressed at the same Frame.
  ///
  template<typename Motion1, typename Motion2, typename Vector3Like>
  inline void classicAcceleration(const MotionDense<Motion1> & spatial_velocity,
                                  const MotionDense<Motion2> & spatial_acceleration,
                                  const Eigen::MatrixBase<Vector3Like> & res)
  {
    PINOCCHIO_EIGEN_CONST_CAST(Vector3Like,res).noalias()
    = spatial_acceleration.linear() + spatial_velocity.angular().cross(spatial_velocity.linear());
  }
  
  ///
  /// \brief Computes the classic acceleration from a given spatial velocity and spatial acceleration.
  ///
  /// \tparam Motion1 type of the input spatial velocity.
  /// \tparam Motion2 type of the input spatial acceleration.
  ///
  /// \param[in] spatial_velocity input spatial velocity.
  /// \param[in] spatial_acceleration input spatial acceleration.
  ///
  /// \remarks To be valid, the spatial velocity and the spatial acceleration have to be expressed at the same Frame.
  ///
  template<typename Motion1, typename Motion2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename Motion2::Vector3)
  classicAcceleration(const MotionDense<Motion1> & spatial_velocity,
                      const MotionDense<Motion2> & spatial_acceleration)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename Motion2::Vector3) ReturnType;
    ReturnType res;
    classicAcceleration(spatial_velocity,spatial_acceleration,res);
    return res;
  }
  
  ///
  /// \brief Computes the classic acceleration of a given frame B
  ///        knowing the spatial velocity and spatial acceleration of a frame A
  ///        and the relative placement between these two frames.
  ///
  /// \tparam Motion1 type of the input spatial velocity.
  /// \tparam Motion2 type of the input spatial acceleration.
  /// \tparam SE3Scalar Scalar type of the SE3 object.
  /// \tparam SE3Options Options of the SE3 object.
  /// \tparam Vector3Like type of the return type (a type similar to a 3D vector).
  ///
  /// \param[in] spatial_velocity input spatial velocity.
  /// \param[in] spatial_acceleration input spatial acceleration.
  /// \param[in] placement relative placement betwen the frame A and the frame B.
  /// \param[out] res computed classic acceleration.
  ///
  template<typename Motion1, typename Motion2, typename SE3Scalar, int SE3Options, typename Vector3Like>
  inline void classicAcceleration(const MotionDense<Motion1> & spatial_velocity,
                                  const MotionDense<Motion2> & spatial_acceleration,
                                  const SE3Tpl<SE3Scalar,SE3Options> & placement,
                                  const Eigen::MatrixBase<Vector3Like> & res)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename Motion1::LinearType) Vector3;
    
    const Vector3 linear_velocity_frame_B
    = spatial_velocity.linear()
    + spatial_velocity.angular().cross(placement.translation());
    
    const Vector3 linear_acceleration_frame_B // expressed in the coordinate frame A
    = spatial_acceleration.linear()
    + spatial_velocity.angular().cross(linear_velocity_frame_B);
    
    PINOCCHIO_EIGEN_CONST_CAST(Vector3Like,res).noalias()
    = placement.rotation().transpose() * (linear_acceleration_frame_B - placement.translation().cross(spatial_acceleration.angular()));
  }
  
  ///
  /// \brief Computes the classic acceleration of a given frame B
  ///        knowing the spatial velocity and spatial acceleration of a frame A
  ///        and the relative placement between these two frames.
  ///
  /// \tparam Motion1 type of the input spatial velocity.
  /// \tparam Motion2 type of the input spatial acceleration.
  /// \tparam SE3Scalar Scalar type of the SE3 object.
  /// \tparam SE3Options Options of the SE3 object.
  ///
  /// \param[in] spatial_velocity input spatial velocity.
  /// \param[in] spatial_acceleration input spatial acceleration.
  /// \param[in] placement relative placement betwen the frame A and the frame B.
  ///
  template<typename Motion1, typename Motion2, typename SE3Scalar, int SE3Options>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename Motion2::Vector3)
  classicAcceleration(const MotionDense<Motion1> & spatial_velocity,
                      const MotionDense<Motion2> & spatial_acceleration,
                      const SE3Tpl<SE3Scalar,SE3Options> & placement)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename Motion2::Vector3) ReturnType;
    ReturnType res;
    classicAcceleration(spatial_velocity,spatial_acceleration,placement,res);
    return res;
  }
}

#endif // ifndef __pinocchio_spatial_classic_acceleration_hpp__
