//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_spatial_classic_acceleration_hpp__
#define __pinocchio_spatial_classic_acceleration_hpp__

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
}

#endif // ifndef __pinocchio_spatial_classic_acceleration_hpp__
