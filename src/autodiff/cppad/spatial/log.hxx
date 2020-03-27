//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_autodiff_cppad_spatial_log_hxx__
#define __pinocchio_autodiff_cppad_spatial_log_hxx__

#include "pinocchio/spatial/log.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace pinocchio
{
  /// \brief Template specialization of log3 function
  template<typename _Scalar>
  struct log3_impl< CppAD::AD<_Scalar> >
  {
    template<typename Matrix3Like, typename Vector3Out>
    static void run(const Eigen::MatrixBase<Matrix3Like> & R,
                    typename Matrix3Like::Scalar & theta,
                    const Eigen::MatrixBase<Vector3Out> & res)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, R, 3, 3);
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector3Out, res, 3, 1);
      // TODO: add static_assert<is_same_type<_Scalar,Scalar>

      typedef typename Matrix3Like::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options> Vector3;
    
      static const Scalar PI_value = PI<_Scalar>();
      
      const Scalar tr = R.trace();
      theta = CppAD::CondExpLt<_Scalar>(tr, Scalar(-1),
                                        PI_value,
                                        CppAD::CondExpGt<_Scalar>(tr, Scalar(3),
                                                                  Scalar(0),
                                                                  math::acos((tr - Scalar(1))/Scalar(2))));
      
      const Scalar cphi = -(tr - Scalar(1))/Scalar(2);
      const Scalar beta = theta*theta / (Scalar(1) + cphi);
      const Vector3 tmp((R.diagonal().array() + cphi) * beta);
      const Scalar t = CppAD::CondExpGt<_Scalar>(theta,
                                                 TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                 theta / sin(theta),
                                                 Scalar(1)) / Scalar(2);
      
      Vector3Out & res_ = PINOCCHIO_EIGEN_CONST_CAST(Vector3Out,res);
      
      res_(0) = CppAD::CondExpGe<_Scalar>(theta,
                                          PI_value-Scalar(1e-2),
                                          CppAD::CondExpGt<_Scalar>(R (2, 1), R (1, 2), Scalar(1), Scalar(-1)) *
                                          CppAD::CondExpGt<_Scalar>(tmp[0], Scalar(0), sqrt(tmp[0]), Scalar(0)),
                                          t * (R (2, 1) - R (1, 2)));
      
      res_(1) = CppAD::CondExpGe<_Scalar>(theta,
                                          PI_value-Scalar(1e-2),
                                          CppAD::CondExpGt<_Scalar>(R (0, 2), R (2, 0), Scalar(1), Scalar(-1)) *
                                          CppAD::CondExpGt<_Scalar>(tmp[1], Scalar(0), sqrt(tmp[1]), Scalar(0)),
                                          t * (R (0, 2) - R (2, 0)));
      
      res_(2) = CppAD::CondExpGe<_Scalar>(theta,
                                          PI_value-Scalar(1e-2),
                                          CppAD::CondExpGt<_Scalar>(R (1, 0), R (0, 1), Scalar(1), Scalar(-1)) *
                                          CppAD::CondExpGt<_Scalar>(tmp[2], Scalar(0), sqrt(tmp[2]), Scalar(0)),
                                          t * (R (1, 0) - R (0, 1)));
    }
  };

  /// \brief Template specialization of Jlog3 function
  template<typename _Scalar>
  struct Jlog3_impl< CppAD::AD<_Scalar> >
  {
    template<typename Scalar, typename Vector3Like, typename Matrix3Like>
    static void run(const Scalar & theta,
                    const Eigen::MatrixBase<Vector3Like> & log,
                    const Eigen::MatrixBase<Matrix3Like> & Jlog)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector3Like,  log, 3, 1);
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, Jlog, 3, 3);
      // TODO: add static_assert<is_same_type<_Scalar,Scalar>
       
      Matrix3Like & Jlog_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,Jlog);

      Scalar ct,st; SINCOS(theta,&st,&ct);
      const Scalar st_1mct = st/(Scalar(1)-ct);

      const Scalar alpha = CppAD::CondExpLt<_Scalar>(theta,
                                                     TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                     Scalar(1)/Scalar(12) + theta*theta / Scalar(720),
                                                     Scalar(1)/(theta*theta) - st_1mct/(Scalar(2)*theta));
      
      Jlog_.noalias() = alpha * log * log.transpose();
      
      Jlog_.diagonal()[0] = CppAD::CondExpLt<_Scalar>(theta,
                                                      TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                      Jlog_.diagonal()[0] + Scalar(0.5) * (2 - theta*theta / Scalar(6)),
                                                      Jlog_.diagonal()[0] + Scalar(0.5) * (theta*st_1mct));
      Jlog_.diagonal()[1] = CppAD::CondExpLt<_Scalar>(theta,
                                                      TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                      Jlog_.diagonal()[1] + Scalar(0.5) * (2 - theta*theta / Scalar(6)),
                                                      Jlog_.diagonal()[1] + Scalar(0.5) * (theta*st_1mct));
      Jlog_.diagonal()[2] = CppAD::CondExpLt<_Scalar>(theta,
                                                      TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                      Jlog_.diagonal()[2] + Scalar(0.5) * (2 - theta*theta / Scalar(6)),
                                                      Jlog_.diagonal()[2] + Scalar(0.5) * (theta*st_1mct));

      addSkew(Scalar(0.5) * log, Jlog_);
    }
  };

  /// \brief Template specialization of log6 function
  template<typename _Scalar>
  struct log6_impl< CppAD::AD<_Scalar> >
  {
    template<typename Scalar, int Options, typename MotionDerived>
    static void run(const SE3Tpl<Scalar,Options> & M,
                    MotionDense<MotionDerived> & mout)
    {
      typedef SE3Tpl<Scalar,Options> SE3;
      typedef typename SE3::Vector3 Vector3;
      
      typename SE3::ConstAngularRef R = M.rotation();
      typename SE3::ConstLinearRef p = M.translation();
      
      Scalar t;
      const Vector3 w(log3(R,t)); // t in [0,π]
      const Scalar t2 = t*t;
      Scalar alpha, beta;
      Scalar st,ct; SINCOS(t,&st,&ct);
      
      alpha = CppAD::CondExpLt<_Scalar>(t,
                                        TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                        Scalar(1) - t2/Scalar(12) - t2*t2/Scalar(720),
                                        t*st/(Scalar(2)*(Scalar(1)-ct)));
      beta = CppAD::CondExpLt<_Scalar>(t,
                                       TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                       Scalar(1)/Scalar(12) + t2/Scalar(720),
                                       Scalar(1)/t2 - st/(Scalar(2)*t*(Scalar(1)-ct)));
      
      mout.linear().noalias() = alpha * p - Scalar(0.5) * w.cross(p) + beta * w.dot(p) * w;
      mout.angular() = w;
    }
  };

  /// \brief Template specialization of Jlog6 function
  template<typename _Scalar>
  struct Jlog6_impl< CppAD::AD<_Scalar> >
  {
    template<typename Scalar, int Options, typename Matrix6Like>
    static void run(const SE3Tpl<Scalar, Options> & M,
                    const Eigen::MatrixBase<Matrix6Like> & Jlog)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix6Like, Jlog, 6, 6);
      // TODO: add static_assert<is_same_type<_Scalar,Scalar>

      typedef SE3Tpl<Scalar,Options> SE3;
      typedef typename SE3::Vector3 Vector3;
      Matrix6Like & Jlog_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,Jlog);

      typename SE3::ConstAngularRef R = M.rotation();
      typename SE3::ConstLinearRef p = M.translation();

      Scalar t;
      Vector3 w(log3(R,t));

      // value is decomposed as following:
      // value = [ A, B;
      //           C, D ]
      typedef Eigen::Block<Matrix6Like,3,3> Block33;
      Block33 A = Jlog_.template topLeftCorner<3,3>();
      Block33 B = Jlog_.template topRightCorner<3,3>();
      Block33 C = Jlog_.template bottomLeftCorner<3,3>();
      Block33 D = Jlog_.template bottomRightCorner<3,3>();

      Jlog3(t, w, A);
      D = A;

      const Scalar t2 = t*t;
      Scalar beta, beta_dot_over_theta;

      const Scalar tinv = Scalar(1)/t, t2inv = tinv*tinv;
      Scalar st,ct; SINCOS (t, &st, &ct);
      const Scalar inv_2_2ct = Scalar(1)/(Scalar(2)*(Scalar(1)-ct));
      
      beta = CppAD::CondExpLt<_Scalar>(t,
                                       TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                       Scalar(1)/Scalar(12) + t2/Scalar(720),
                                       t2inv - st*tinv*inv_2_2ct);
      
      beta_dot_over_theta = CppAD::CondExpLt<_Scalar>(t,
                                                      TaylorSeriesExpansion<_Scalar>::template precision<3>(),
                                                      Scalar(1)/Scalar(360),
                                                      -Scalar(2)*t2inv*t2inv + (Scalar(1) + st*tinv) * t2inv * inv_2_2ct);

      Scalar wTp = w.dot(p);

      Vector3 v3_tmp(  (beta_dot_over_theta*wTp)*w
                     - (t2*beta_dot_over_theta+Scalar(2)*beta)*p);
      // C can be treated as a temporary variable
      C.noalias() = v3_tmp * w.transpose();
      C.noalias() += beta * w * p.transpose();
      C.diagonal().array() += wTp * beta;
      addSkew(Scalar(.5)*p,C);

      B.noalias() = C * A;
      C.setZero();
    }
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_autodiff_cppad_spatial_log_hxx__
