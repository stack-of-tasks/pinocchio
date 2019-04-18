//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_special_orthogonal_operation_hpp__
#define __pinocchio_special_orthogonal_operation_hpp__

#include <limits>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/multibody/liegroup/liegroup-base.hpp>

namespace pinocchio
{
  template<int Dim, typename Scalar, int Options = 0>
  struct SpecialOrthogonalOperationTpl
  {};
  
  template<int Dim, typename Scalar, int Options>
  struct traits< SpecialOrthogonalOperationTpl<Dim,Scalar,Options> >
  {};

  template<typename _Scalar, int _Options>
  struct traits< SpecialOrthogonalOperationTpl<2,_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
      NQ = 2,
      NV = 1
    };
  };

  template<typename _Scalar, int _Options >
  struct traits<SpecialOrthogonalOperationTpl<3,_Scalar,_Options> > {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
      NQ = 4,
      NV = 3
    };
  };

  template<typename _Scalar, int _Options>
  struct SpecialOrthogonalOperationTpl<2,_Scalar,_Options>
  : public LieGroupBase< SpecialOrthogonalOperationTpl<2,_Scalar,_Options> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(SpecialOrthogonalOperationTpl);
    typedef Eigen::Matrix<Scalar,2,2> Matrix2;

    template<typename Matrix2Like>
    static typename Matrix2Like::Scalar
    log(const Eigen::MatrixBase<Matrix2Like> & R)
    {
      
      typedef typename Matrix2Like::Scalar Scalar;
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like,2,2);
      
      Scalar theta;
      const Scalar tr = R.trace();
      const bool pos = (R (1, 0) > Scalar(0));
      const Scalar PI_value = PI<Scalar>();
      if (tr > Scalar(2))       theta = Scalar(0); // acos((3-1)/2)
      else if (tr < Scalar(-2)) theta = (pos ? PI_value : -PI_value); // acos((-1-1)/2)
      // Around 0, asin is numerically more stable than acos because
      // acos(x) = PI/2 - x and asin(x) = x (the precision of x is not lost in PI/2).
      else if (tr > Scalar(2) - 1e-2) theta = asin ((R(1,0) - R(0,1)) / Scalar(2));
      else              theta = (pos ? acos (tr/Scalar(2)) : -acos(tr/Scalar(2)));
      assert (theta == theta); // theta != NaN
      assert ((cos (theta) * R(0,0) + sin (theta) * R(1,0) > 0) &&
              (cos (theta) * R(1,0) - sin (theta) * R(0,0) < 1e-6));
      return theta;
    }

    template<typename Matrix2Like>
    static typename Matrix2Like::Scalar
    Jlog(const Eigen::MatrixBase<Matrix2Like> &)
    {
      typedef typename Matrix2Like::Scalar Scalar;
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like,2,2);
      return (Scalar)1;
    }

    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    static Index nq()
    {
      return NQ;
    }
    
    /// Get dimension of Lie Group tangent space
    static Index nv()
    {
      return NV;
    }

    static ConfigVector_t neutral()
    {
      ConfigVector_t n; n << Scalar(1), Scalar(0);
      return n;
    }

    static std::string name()
    {
      return std::string("SO(2)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      if (q0 == q1) {
        PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d).setZero();
        return;
      }
      Matrix2 R; // R0.transpose() * R1;
      R(0,0) = R(1,1) = q0.dot(q1);
      R(1,0) = q0(0) * q1(1) - q0(1) * q1(0);
      R(0,1) = - R(1,0);
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d)[0] = log(R);
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl (const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<JacobianOut_t>& J) const
    {
      Matrix2 R; // R0.transpose() * R1;
      R(0,0) = R(1,1) = q0.dot(q1);
      R(1,0) = q0(0) * q1(1) - q0(1) * q1(0);
      R(0,1) = - R(1,0);

      Scalar w (Jlog(R));
      PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).coeffRef(0,0) = ((arg==ARG0) ? -w : w);
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t & out = PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout);

      const Scalar & ca = q(0);
      const Scalar & sa = q(1);
      const Scalar & omega = v(0);

      Scalar cosOmega,sinOmega; SINCOS(omega, &sinOmega, &cosOmega);
      // TODO check the cost of atan2 vs SINCOS

      out << cosOmega * ca - sinOmega * sa,
             sinOmega * ca + cosOmega * sa;
      // First order approximation of the normalization of the unit complex
      // See quaternion::firstOrderNormalize for equations.
      const Scalar norm2 = out.squaredNorm();
      out *= (3 - norm2) / 2;
    }
    
    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      Jout << -q[1], q[0];
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      Jout(0,0) = 1;
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      Jout(0,0) = 1;
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar& u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      ConfigOut_t & out = PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout);

      assert ( std::abs(q0.norm() - 1) < 1e-8 && "initial configuration not normalized");
      assert ( std::abs(q1.norm() - 1) < 1e-8 && "final configuration not normalized");
      Scalar cosTheta = q0.dot(q1);
      Scalar sinTheta = q0(0)*q1(1) - q0(1)*q1(0);
      Scalar theta = atan2(sinTheta, cosTheta);
      assert (fabs (sin (theta) - sinTheta) < 1e-8);
      
      const Scalar PI_value = PI<Scalar>();

      if (fabs (theta) > 1e-6 && fabs (theta) < PI_value - 1e-6)
      {
        out = (sin ((1-u)*theta)/sinTheta) * q0
            + (sin (   u *theta)/sinTheta) * q1;
      }
      else if (fabs (theta) < 1e-6) // theta = 0
      {
        out = (1-u) * q0 + u * q1;
      }
      else // theta = +-PI
      {
        Scalar theta0 = atan2 (q0(1), q0(0));
        SINCOS(theta0,&out[1],&out[0]);
      }
    }

    // template <class ConfigL_t, class ConfigR_t>
    // static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       // const Eigen::MatrixBase<ConfigR_t> & q1)
    
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t& qout_ = (const_cast< Eigen::MatrixBase<Config_t>& >(qout)).derived();
      qout_.normalize();
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      Config_t & out = PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout);
      
      const Scalar PI_value = PI<Scalar>();
      const Scalar angle = -PI_value + Scalar(2)* PI_value * ((Scalar)rand())/RAND_MAX;
      SINCOS(angle, &out(1), &out(0));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                  const Eigen::MatrixBase<ConfigR_t> &,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      random_impl(qout);
    }
  }; // struct SpecialOrthogonalOperationTpl<2,_Scalar,_Options>

  template<typename _Scalar, int _Options>
  struct SpecialOrthogonalOperationTpl<3,_Scalar,_Options>
  : public LieGroupBase< SpecialOrthogonalOperationTpl<3,_Scalar,_Options> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(SpecialOrthogonalOperationTpl);

    typedef Eigen::Quaternion<Scalar> Quaternion_t;
    typedef Eigen::Map<      Quaternion_t> QuaternionMap_t;
    typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;

    /// Get dimension of Lie Group vector representation
    ///
    /// For instance, for SO(3), the dimension of the vector representation is
    /// 4 (quaternion) while the dimension of the tangent space is 3.
    static Index nq()
    {
      return NQ;
    }
    /// Get dimension of Lie Group tangent space
    static Index nv()
    {
      return NV;
    }

    static ConfigVector_t neutral()
    {
      ConfigVector_t n; n.setZero (); n[3] = Scalar(1);
      return n;
    }

    static std::string name()
    {
      return std::string("SO(3)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      if (q0 == q1) {
        PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d).setZero();
        return;
      }
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d)
        = log3((p0.matrix().transpose() * p1.matrix()).eval());
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl (const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<JacobianOut_t>& J) const
    {
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      Eigen::Matrix<Scalar, 3, 3> R = p0.matrix().transpose() * p1.matrix();

      if (arg == ARG0) {
        JacobianMatrix_t J1;
        Jlog3 (R, J1);

        PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).noalias() = - J1 * R.transpose();
      } else if (arg == ARG1) {
        Jlog3 (R, J);
      }
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConstQuaternionMap_t quat(q.derived().data());
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout).data());

      Quaternion_t pOmega; quaternion::exp3(v,pOmega);
      quat_map = quat * pOmega;
      quaternion::firstOrderNormalize(quat_map);
    }
    
    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Config_t) ConfigPlainType;
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Jacobian_t) JacobianPlainType;
      typedef typename ConfigPlainType::Scalar Scalar;
      typedef SE3Tpl<Scalar,ConfigPlainType::Options> SE3;
      typedef typename SE3::Vector3 Vector3;
      typedef typename SE3::Matrix3 Matrix3;

      ConstQuaternionMap_t quat_map(q.derived().data());
      Eigen::Matrix<Scalar,NQ,NV,JacobianPlainType::Options|Eigen::RowMajor> Jexp3QuatCoeffWise;
      
      Scalar theta;
      Vector3 v = quaternion::log3(quat_map,theta);
      quaternion::Jexp3CoeffWise(v,Jexp3QuatCoeffWise);
      Matrix3 Jlog;
      Jlog3(theta,v,Jlog);
      
//      if(quat_map.w() >= 0.) // comes from the log3 for quaternions which may change the sign.
      if(quat_map.coeffs()[3] >= 0.) // comes from the log3 for quaternions which may change the sign.
        PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).noalias() = Jexp3QuatCoeffWise * Jlog;
      else
        PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).noalias() = -Jexp3QuatCoeffWise * Jlog;
        
//      Jexp3(quat_map,PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).template topLeftCorner<NQ,NV>());
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & v,
                                   const Eigen::MatrixBase<JacobianOut_t>& J)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      Jout = exp3(-v);
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & v,
                                   const Eigen::MatrixBase<JacobianOut_t> & J)
    {
      Jexp3(v, J.derived());
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar & u,
                                 const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      ConstQuaternionMap_t p0 (q0.derived().data());
      ConstQuaternionMap_t p1 (q1.derived().data());
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout).data());

      quat_map = p0.slerp(u, p1);
    }

    template <class ConfigL_t, class ConfigR_t>
    static Scalar squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       const Eigen::MatrixBase<ConfigR_t> & q1)
    {
      TangentVector_t t;
      difference_impl(q0, q1, t);
      return t.squaredNorm();
    }
    
    template <class Config_t>
    static void normalize_impl(const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t & qout_ = PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout);
      qout_.normalize();
    }

    template <class Config_t>
    void random_impl(const Eigen::MatrixBase<Config_t> & qout) const
    {
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout).data());
      quaternion::uniformRandom(quat_map);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                  const Eigen::MatrixBase<ConfigR_t> &,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      random_impl(qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      ConstQuaternionMap_t quat1(q0.derived().data());
      ConstQuaternionMap_t quat2(q1.derived().data());

      return quaternion::defineSameRotation(quat1,quat2,prec);
    }
  }; // struct SpecialOrthogonalOperationTpl<3,_Scalar,_Options>
  
} // namespace pinocchio

#endif // ifndef __pinocchio_special_orthogonal_operation_hpp__
