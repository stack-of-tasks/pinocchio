//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_liegroup_special_orthogonal_operation_hpp__
#define __pinocchio_multibody_liegroup_special_orthogonal_operation_hpp__

#include <limits>

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/utils/static-if.hpp"

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
    typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

    template<typename Matrix2Like>
    static typename Matrix2Like::Scalar
    log(const Eigen::MatrixBase<Matrix2Like> & R)
    {
      typedef typename Matrix2Like::Scalar Scalar;
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like,2,2);
      
      const Scalar tr = R.trace();
      
      static const Scalar PI_value = PI<Scalar>();
      
      using internal::if_then_else;
      Scalar theta =
      if_then_else(internal::GT, tr, Scalar(2),
                   Scalar(0), // then
                   if_then_else(internal::LT, tr, Scalar(-2),
                                if_then_else(internal::GE, R (1, 0), Scalar(0),
                                             PI_value, -PI_value), // then
                                if_then_else(internal::GT, tr, Scalar(2) - Scalar(1e-2),
                                             asin((R(1,0) - R(0,1)) / Scalar(2)), // then
                                             if_then_else(internal::GE, R (1, 0), Scalar(0),
                                                          acos(tr/Scalar(2)), // then
                                                          -acos(tr/Scalar(2))
                                                          )
                                             )
                                )
                   );
      
                                          
//      const bool pos = (R (1, 0) > Scalar(0));
//      if (tr > Scalar(2))       theta = Scalar(0); // acos((3-1)/2)
//      else if (tr < Scalar(-2)) theta = (pos ? PI_value : -PI_value); // acos((-1-1)/2)
      // Around 0, asin is numerically more stable than acos because
      // acos(x) = PI/2 - x and asin(x) = x (the precision of x is not lost in PI/2).
//      else if (tr > Scalar(2) - 1e-2) theta = asin ((R(1,0) - R(0,1)) / Scalar(2));
//      else              theta = (pos ? acos (tr/Scalar(2)) : -acos(tr/Scalar(2)));
      assert (theta == theta); // theta != NaN
//      assert ((cos (theta) * R(0,0) + sin (theta) * R(1,0) > 0) &&
//              (cos (theta) * R(1,0) - sin (theta) * R(0,0) < 1e-6));
      return theta;
    }

    template<typename Matrix2Like>
    static typename Matrix2Like::Scalar
    Jlog(const Eigen::MatrixBase<Matrix2Like> &)
    {
      typedef typename Matrix2Like::Scalar Scalar;
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like,2,2);
      return Scalar(1);
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
      Matrix2 R; // R0.transpose() * R1;
      R(0,0) = R(1,1) = q0.dot(q1);
      R(1,0) = q0(0) * q1(1) - q0(1) * q1(0);
      R(0,1) = - R(1,0);
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d)[0] = log(R);
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Eigen::MatrixBase<JacobianOut_t> & J) const
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

      const Scalar ca = q(0);
      const Scalar sa = q(1);
      const Scalar & omega = v(0);

      Scalar cosOmega,sinOmega; SINCOS(omega, &sinOmega, &cosOmega);
      // TODO check the cost of atan2 vs SINCOS

      out << cosOmega * ca - sinOmega * sa,
             sinOmega * ca + cosOmega * sa;
      // First order approximation of the normalization of the unit complex
      // See quaternion::firstOrderNormalize for equations.
      const Scalar norm2 = out.squaredNorm();
      out *= (3 - norm2) / 2;
      assert (isNormalized(out));
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
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t> & J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      switch(op)
        {
        case SETTO:
          Jout(0,0) = Scalar(1);
          break;
        case ADDTO:
          Jout(0,0) += Scalar(1);
          break;
        case RMTO:
          Jout(0,0) -= Scalar(1);
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                   const Eigen::MatrixBase<JacobianOut_t> & J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      switch(op)
        {
        case SETTO:
          Jout(0,0) = Scalar(1);
          break;
        case ADDTO:
          Jout(0,0) += Scalar(1);
          break;
        case RMTO:
          Jout(0,0) -= Scalar(1);
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }      
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & Jout) const
    {
      PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,Jout) = Jin;
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & Jout) const
    {
      PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,Jout) = Jin;
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                     const Eigen::MatrixBase<Jacobian_t> & /*J*/) const {}

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & /*v*/,
                                     const Eigen::MatrixBase<Jacobian_t> & /*J*/) const {}
    

    
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

    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t> & qout)
    {
      Config_t & qout_ = PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout).derived();
      qout_.normalize();
    }

    template <class Config_t>
    static bool isNormalized_impl (const Eigen::MatrixBase<Config_t> & qin,
                                   const Scalar& prec)
    {
      const Scalar norm = qin.norm();
      using std::abs;
      return abs(norm - Scalar(1.0)) < prec;
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
                                  const Eigen::MatrixBase<ConfigOut_t> & qout) const
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
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
    
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
      ConstQuaternionMap_t quat0 (q0.derived().data());
      assert(quaternion::isNormalized(quat0,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat1 (q1.derived().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d)
        = quaternion::log3(Quaternion_t(quat0.conjugate() * quat1));
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl (const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      typedef typename SE3::Matrix3 Matrix3;

      ConstQuaternionMap_t quat0 (q0.derived().data());
      assert(quaternion::isNormalized(quat0,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat1 (q1.derived().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      const Quaternion_t quat_diff = quat0.conjugate() * quat1;

      if (arg == ARG0) {
        JacobianMatrix_t J1;
        quaternion::Jlog3(quat_diff, J1);
        const Matrix3 R = (quat_diff).matrix();

        PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J).noalias() = - J1 * R.transpose();
      } else if (arg == ARG1) {
        quaternion::Jlog3(quat_diff, J);
      }
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConstQuaternionMap_t quat(q.derived().data());
      assert(quaternion::isNormalized(quat,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout).data());

      Quaternion_t pOmega; quaternion::exp3(v,pOmega);
      quat_map = quat * pOmega;
      quaternion::firstOrderNormalize(quat_map);
      assert(quaternion::isNormalized(quat_map,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
    }
    
    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Jacobian_t) JacobianPlainType;
      typedef typename SE3::Vector3 Vector3;
      typedef typename SE3::Matrix3 Matrix3;

      ConstQuaternionMap_t quat_map(q.derived().data());
      assert(quaternion::isNormalized(quat_map,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      Eigen::Matrix<Scalar,NQ,NV,JacobianPlainType::Options|Eigen::RowMajor> Jexp3QuatCoeffWise;
      
      Scalar theta;
      const Vector3 v = quaternion::log3(quat_map,theta);
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
                                   const Eigen::MatrixBase<JacobianOut_t> & J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      switch(op)
        {
        case SETTO:
          Jout = exp3(-v);
          break;
        case ADDTO:
          Jout += exp3(-v);
          break;
        case RMTO:
          Jout -= exp3(-v);
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }      
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t> & v,
                                   const Eigen::MatrixBase<JacobianOut_t> & J,
                                   const AssignmentOperatorType op=SETTO)
    {
      switch(op)
        {
        case SETTO:
          Jexp3<SETTO>(v, J.derived());
          break;
        case ADDTO:
          Jexp3<ADDTO>(v, J.derived());
          break;
        case RMTO:
          Jexp3<RMTO>(v, J.derived());
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }      
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      typedef typename SE3::Matrix3 Matrix3;
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      const Matrix3 Jtmp3 = exp3(-v);
      Jout.noalias() = Jtmp3 * Jin;
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      typedef typename SE3::Matrix3 Matrix3;
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      Matrix3 Jtmp3;
      Jexp3<SETTO>(v, Jtmp3);
      Jout.noalias() = Jtmp3 * Jin;
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J_out) const
    {
      typedef typename SE3::Matrix3 Matrix3;
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J_out);
      const Matrix3 Jtmp3 = exp3(-v);
      Jout = Jtmp3 * Jout;
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J_out) const
    {
      typedef typename SE3::Matrix3 Matrix3;
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J_out);
      Matrix3 Jtmp3;
      Jexp3<SETTO>(v, Jtmp3);
      Jout = Jtmp3 * Jout;
    }

    
    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    static void interpolate_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                 const Eigen::MatrixBase<ConfigR_t> & q1,
                                 const Scalar & u,
                                 const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConstQuaternionMap_t quat0 (q0.derived().data());
      assert(quaternion::isNormalized(quat0,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat1 (q1.derived().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout).data());

      quat_map = quat0.slerp(u, quat1);
      assert(quaternion::isNormalized(quat_map,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
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
    static void normalize_impl(const Eigen::MatrixBase<Config_t> & qout)
    {
      Config_t & qout_ = PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout);
      qout_.normalize();
    }

    template <class Config_t>
    static bool isNormalized_impl (const Eigen::MatrixBase<Config_t> & qin,
                                   const Scalar& prec)
    {
      const Scalar norm = qin.norm();
      using std::abs;
      return abs(norm - Scalar(1.0)) < prec;
    }

    template <class Config_t>
    void random_impl(const Eigen::MatrixBase<Config_t> & qout) const
    {
      QuaternionMap_t quat_map(PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout).data());
      quaternion::uniformRandom(quat_map);
      
      assert(quaternion::isNormalized(quat_map,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> &,
                                  const Eigen::MatrixBase<ConfigR_t> &,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      random_impl(qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      ConstQuaternionMap_t quat1(q0.derived().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat2(q1.derived().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));

      return quaternion::defineSameRotation(quat1,quat2,prec);
    }
  }; // struct SpecialOrthogonalOperationTpl<3,_Scalar,_Options>
  
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_liegroup_special_orthogonal_operation_hpp__
