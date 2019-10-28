//
// Copyright (c) 2016-2019 CNRS, INRIA
//

#ifndef __pinocchio_math_rpy_hpp__
#define __pinocchio_math_rpy_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"
#include "pinocchio/math/sincos.hpp"
#include <boost/type_traits.hpp>

#include <Eigen/Geometry>

namespace pinocchio
{
  namespace rpy
  {
    ///
    /// \brief Convert from Roll, Pitch, Yaw to transformation Matrix
    ///
    /// Given \f$r, p, y\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    /// As this is a specialized implementation, it is expected to be very efficient
    ///
    template<typename Scalar>
    Eigen::Matrix<Scalar,3,3> rpyToMatrix(Scalar r, Scalar p, Scalar y)
    {
      typedef Eigen::Matrix<Scalar,3,3> ResultType;
      ResultType res;

      Scalar sr, cr;
      SINCOS(r,&sr,&cr);
      Scalar sp, cp;
      SINCOS(p,&sp,&cp);
      Scalar sy, cy;
      SINCOS(y,&sy,&cy);

      res <<  cp*cy, cy*sp*sr - cr*sy, sr*sy + cr*cy*sp,
              cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr,
                -sp,            cp*sr,            cp*cr;

      return res;         
    }

    ///
    /// \brief Convert from Roll, Pitch, Yaw to transformation Matrix
    ///
    /// Given a vector \f$(r, p, y)\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    /// As this is a specialized implementation, it is expected to be very efficient
    ///
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    rpyToMatrix(const Eigen::MatrixBase<Vector3Like> & v)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE (Vector3Like, v, 3, 1);

      return rpyToMatrix(v[0],v[1],v[2]);
    }
  } // namespace rpy
}
#endif //#ifndef __pinocchio_math_rpy_hpp__
