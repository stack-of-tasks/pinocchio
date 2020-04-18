//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_math_rpy_hpp__
#define __pinocchio_math_rpy_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"

#include <Eigen/Geometry>

namespace pinocchio
{
  namespace rpy
  {
    ///
    /// \brief Convert from Roll, Pitch, Yaw to rotation Matrix
    ///
    /// Given \f$r, p, y\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    ///
    template<typename Scalar>
    Eigen::Matrix<Scalar,3,3> rpyToMatrix(const Scalar r,
                                          const Scalar p,
                                          const Scalar y)
    {
      typedef Eigen::AngleAxis<Scalar> AngleAxis;
      typedef Eigen::Matrix<Scalar,3,1> Vector3s;
      return (AngleAxis(y, Vector3s::UnitZ())
              * AngleAxis(p, Vector3s::UnitY())
              * AngleAxis(r, Vector3s::UnitX())
             ).toRotationMatrix();
    }

    ///
    /// \brief Convert from Roll, Pitch, Yaw to rotation Matrix
    ///
    /// Given a vector \f$(r, p, y)\f$, the rotation is given as \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    ///
    template<typename Vector3Like>
    Eigen::Matrix<typename Vector3Like::Scalar,3,3,PINOCCHIO_EIGEN_PLAIN_TYPE(Vector3Like)::Options>
    rpyToMatrix(const Eigen::MatrixBase<Vector3Like> & rpy)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector3Like, rpy, 3, 1);
      return rpyToMatrix(rpy[0], rpy[1], rpy[2]);
    }

    ///
    /// \brief Convert from Transformation Matrix to Roll, Pitch, Yaw
    ///
    /// Given a rotation matrix \f$R\f$, the angles \f$r, p, y\f$ are given
    /// so that \f$ R = R_z(y)R_y(p)R_x(r) \f$,
    /// where \f$R_{\alpha}(\theta)\f$ denotes the rotation of \f$\theta\f$ degrees
    /// around axis \f$\alpha\f$.
    /// The angles are guaranteed to be in the ranges \f$r\in[-\pi,\pi]\f$
    /// \f$p\in[-\frac{\pi}{2},\frac{\pi}{2}]\f$ \f$y\in[-\pi,\pi]\f$,
    /// unlike Eigen's eulerAngles() function
    ///
    /// \warning the method assumes \f$R\f$ is a rotation matrix. If it is not, the result is undefined.
    ///
    template<typename Matrix3Like>
    Eigen::Matrix<typename Matrix3Like::Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options>
    matrixToRpy(const Eigen::MatrixBase<Matrix3Like> & R)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, R, 3, 3);
      assert(R.isUnitary() && "R is not a unitary matrix");

      typedef typename Matrix3Like::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options> ReturnType;
      static const Scalar pi = PI<Scalar>();

      ReturnType res = R.eulerAngles(2,1,0).reverse();

      if(res[1] < -pi/2)
        res[1] += 2*pi;

      if(res[1] > pi/2)
      {
        res[1] = pi - res[1];
        if(res[0] < Scalar(0))
          res[0] += pi;
        else
          res[0] -= pi;
        // res[2] > 0 according to Eigen's eulerAngles doc, no need to check its sign
        res[2] -= pi;
      }

      return res;
    }
  } // namespace rpy
}
#endif //#ifndef __pinocchio_math_rpy_hpp__
