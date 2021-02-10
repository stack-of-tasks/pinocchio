//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_spatial_log_hxx__
#define __pinocchio_spatial_log_hxx__

namespace pinocchio
{

  namespace internal
  {
    template<long i0, typename Matrix3, typename Vector3>
    void compute_theta_axis(const typename Matrix3::Scalar & val,
                            const Eigen::MatrixBase<Matrix3> & R,
                            typename Matrix3::Scalar & angle,
                            const Eigen::MatrixBase<Vector3> & _axis)
    {
      typedef typename Matrix3::Scalar Scalar;
      
      static const long i1 = (i0+1) % 3;
      static const long i2 = (i0+2) % 3;
      Vector3 & axis = _axis.const_cast_derived();
      
      const Scalar s = math::sqrt(val) * if_then_else(GE,R.coeff(i2,i1),R.coeff(i1,i2),Scalar(1.),Scalar(-1.));
      axis[i0] = s/Scalar(2);
      axis[i1] = Scalar(1)/(2*s) * (R.coeff(i1,i0) + R.coeff(i0,i1));
      axis[i2] = Scalar(1)/(2*s) * (R.coeff(i2,i0) + R.coeff(i0,i2));
      const Scalar w = Scalar(1)/(2*s)*(R.coeff(i2,i1) - R.coeff(i1,i2));
      
      const Scalar axis_norm = axis.norm();
      angle = 2*math::atan2(axis_norm,w);
      axis /= axis_norm;
    }
  }

  /// \brief Generic evaluation of log3 function
  template<typename _Scalar>
  struct log3_impl
  {
    template<typename Matrix3Like, typename Vector3Out>
    static void run(const Eigen::MatrixBase<Matrix3Like> & R,
                    typename Matrix3Like::Scalar & theta,
                    const Eigen::MatrixBase<Vector3Out> & angle_axis)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, R, 3, 3);
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector3Out, angle_axis, 3, 1);
      using namespace internal;

      typedef typename Matrix3Like::Scalar Scalar;
      typedef Eigen::Matrix<Scalar,3,1,PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix3Like)::Options> Vector3;
    
      const static Scalar PI_value = PI<Scalar>();
      Vector3Out & angle_axis_ = angle_axis.const_cast_derived();
    
      const Scalar tr = R.trace();
      const Scalar cos_value = (tr - Scalar(1))/Scalar(2);
      const Scalar theta_nominal = if_then_else(LE,tr,Scalar(3),
                                                if_then_else(GE,tr,Scalar(-1),
                                                             math::acos(cos_value), // then
                                                             PI_value // else
                                                             ),
                                                Scalar(0) // else
                                                );
      assert(check_expression_if_real<Scalar>(theta_nominal == theta_nominal) && "theta contains some NaN"); // theta != NaN
      
      Vector3 antisymmetric_R; unSkew(R,antisymmetric_R);
      const Scalar norm_antisymmetric_R_squared = antisymmetric_R.squaredNorm();

      // Singular cases when theta == PI
      Vector3 angle_axis_singular; Scalar theta_singular;
      {
        Vector3 val_singular;
        val_singular.array() = 2*R.diagonal().array() - tr + Scalar(1);
        Vector3 axis_0, axis_1, axis_2;
        Scalar theta_0, theta_1, theta_2;
        
        internal::compute_theta_axis<0>(val_singular[0], R, theta_0, axis_0);
        internal::compute_theta_axis<1>(val_singular[1], R, theta_1, axis_1);
        internal::compute_theta_axis<2>(val_singular[2], R, theta_2, axis_2);
        
        theta_singular = if_then_else(GE, val_singular[0], val_singular[1],
                                      if_then_else(GE, val_singular[0], val_singular[2],
                                                   theta_0, theta_2),
                                      if_then_else(GE, val_singular[1], val_singular[2],
                                                   theta_1, theta_2));
        
        for(int k = 0; k < 3; ++k)
          angle_axis_singular[k] = if_then_else(GE, val_singular[0], val_singular[1],
                                                if_then_else(GE, val_singular[0], val_singular[2],
                                                             axis_0[k], axis_2[k]),
                                                if_then_else(GE, val_singular[1], val_singular[2],
                                                             axis_1[k], axis_2[k]));
      }
      
      const Scalar t = if_then_else(GE,theta_nominal,TaylorSeriesExpansion<Scalar>::template precision<2>(),
                                    theta_nominal / sin(theta_nominal), // then
                                    Scalar(1.) + norm_antisymmetric_R_squared/Scalar(6) + norm_antisymmetric_R_squared*norm_antisymmetric_R_squared*Scalar(3)/Scalar(40) // else
                                    );
      
      theta = if_then_else(GE,cos_value,Scalar(-1.) + TaylorSeriesExpansion<Scalar>::template precision<2>(),
                           theta_nominal, theta_singular);
      
      for(int k = 0; k < 3; ++k)
        angle_axis_[k] = if_then_else(GE,cos_value,Scalar(-1.) + TaylorSeriesExpansion<Scalar>::template precision<2>(),
                                      t*antisymmetric_R[k], theta_singular*angle_axis_singular[k]);

    }
  };

  /// \brief Generic evaluation of Jlog3 function
  template<typename _Scalar>
  struct Jlog3_impl
  {
    template<typename Scalar, typename Vector3Like, typename Matrix3Like>
    static void run(const Scalar & theta,
                    const Eigen::MatrixBase<Vector3Like> & log,
                    const Eigen::MatrixBase<Matrix3Like> & Jlog)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector3Like,  log, 3, 1);
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, Jlog, 3, 3);
       
      using namespace internal;
      Scalar ct,st; SINCOS(theta,&st,&ct);
      const Scalar st_1mct = st/(Scalar(1)-ct);
      
      const Scalar alpha = if_then_else(LT,theta,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                        Scalar(1)/Scalar(12) + theta*theta / Scalar(720), // then
                                        Scalar(1)/(theta*theta) - st_1mct/(Scalar(2)*theta) // else
                                        );
      
      const Scalar diag_value = if_then_else(LT,theta,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                             Scalar(0.5) * (2 - theta*theta / Scalar(6)), // then
                                             Scalar(0.5) * (theta * st_1mct) // else
                                             );
        
      Matrix3Like & Jlog_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,Jlog);
      Jlog_.noalias() = alpha * log * log.transpose();
      Jlog_.diagonal().array() += diag_value;
        
      // Jlog += r_{\times}/2
      addSkew(Scalar(0.5) * log, Jlog_);
    }
  };

  /// \brief Generic evaluation of log6 function
  template<typename _Scalar>
  struct log6_impl
  {
    template<typename Scalar, int Options, typename MotionDerived>
    static void run(const SE3Tpl<Scalar,Options> & M,
                    MotionDense<MotionDerived> & mout)
    {
      typedef SE3Tpl<Scalar,Options> SE3;
      typedef typename SE3::Vector3 Vector3;

      typename SE3::ConstAngularRef R = M.rotation();
      typename SE3::ConstLinearRef p = M.translation();
      
      using namespace internal;
      
      Vector3 antisymmetric_R; unSkew(R,antisymmetric_R);
      const Scalar norm_antisymmetric_R_squared = antisymmetric_R.squaredNorm();
      
      Scalar theta;
      const Vector3 w(log3(R,theta)); // t in [0,π]
      const Scalar & t2 = norm_antisymmetric_R_squared;

      Scalar st,ct; SINCOS(theta,&st,&ct);
      const Scalar alpha = if_then_else(LT,theta,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                        Scalar(1) - t2/Scalar(12) - t2*t2/Scalar(720), // then
                                        theta*st/(Scalar(2)*(Scalar(1)-ct)) // else
                                        );
      
      const Scalar beta = if_then_else(LT,theta,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                       Scalar(1)/Scalar(12) + t2/Scalar(720), // then
                                       Scalar(1)/(theta*theta) - st/(Scalar(2)*theta*(Scalar(1)-ct)) // else
                                       );
      
      mout.linear().noalias() = alpha * p - Scalar(0.5) * w.cross(p) + (beta * w.dot(p)) * w;
      mout.angular() = w;
    }
  };

  template<typename _Scalar>
  struct Jlog6_impl
  {
    template<typename Scalar, int Options, typename Matrix6Like>
    static void run(const SE3Tpl<Scalar, Options> & M,
                    const Eigen::MatrixBase<Matrix6Like> & Jlog)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix6Like, Jlog, 6, 6);

      typedef SE3Tpl<Scalar,Options> SE3;
      typedef typename SE3::Vector3 Vector3;
      Matrix6Like & value = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,Jlog);

      typename SE3::ConstAngularRef R = M.rotation();
      typename SE3::ConstLinearRef p = M.translation();
      
      using namespace internal;

      Scalar t;
      Vector3 w(log3(R,t));

      // value is decomposed as following:
      // value = [ A, B;
      //           C, D ]
      typedef Eigen::Block<Matrix6Like,3,3> Block33;
      Block33 A = value.template topLeftCorner<3,3>();
      Block33 B = value.template topRightCorner<3,3>();
      Block33 C = value.template bottomLeftCorner<3,3>();
      Block33 D = value.template bottomRightCorner<3,3>();

      Jlog3(t, w, A);
      D = A;

      const Scalar t2 = t*t;
      const Scalar tinv = Scalar(1)/t,
                   t2inv = tinv*tinv;
      
      Scalar st,ct; SINCOS (t, &st, &ct);
      const Scalar inv_2_2ct = Scalar(1)/(Scalar(2)*(Scalar(1)-ct));

      const Scalar beta = if_then_else(LT,t,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                       Scalar(1)/Scalar(12) + t2/Scalar(720), // then
                                       t2inv - st*tinv*inv_2_2ct // else
                                       );
      
      const Scalar beta_dot_over_theta = if_then_else(LT,t,TaylorSeriesExpansion<Scalar>::template precision<3>(),
                                                      Scalar(1)/Scalar(360), // then
                                                      -Scalar(2)*t2inv*t2inv + (Scalar(1) + st*tinv) * t2inv * inv_2_2ct // else
                                                      );

      const Scalar wTp = w.dot(p);
      const Vector3 v3_tmp((beta_dot_over_theta*wTp)*w - (t2*beta_dot_over_theta+Scalar(2)*beta)*p);
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

#endif // ifndef __pinocchio_spatial_log_hxx__
