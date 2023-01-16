//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_liegroup_special_euclidean_operation_hpp__
#define __pinocchio_multibody_liegroup_special_euclidean_operation_hpp__

#include <limits>

#include "pinocchio/macros.hpp"
#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/utils/static-if.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/liegroup/liegroup-base.hpp"

#include "pinocchio/multibody/liegroup/vector-space.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"
#include "pinocchio/multibody/liegroup/special-orthogonal.hpp"

namespace pinocchio
{
  template<int Dim, typename Scalar, int Options = 0>
  struct SpecialEuclideanOperationTpl
  {};
  
  template<int Dim, typename Scalar, int Options>
  struct traits< SpecialEuclideanOperationTpl<Dim,Scalar,Options> >
  {};

  template<typename _Scalar, int _Options>
  struct traits< SpecialEuclideanOperationTpl<2,_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
      NQ = 4,
      NV = 3
    };
  };

  // SE(2)
  template<typename _Scalar, int _Options>
  struct SpecialEuclideanOperationTpl<2,_Scalar,_Options>
  : public LieGroupBase <SpecialEuclideanOperationTpl<2,_Scalar,_Options> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(SpecialEuclideanOperationTpl);
    
    typedef VectorSpaceOperationTpl<2,Scalar,Options>       R2_t;
    typedef SpecialOrthogonalOperationTpl<2,Scalar,Options> SO2_t;
    typedef CartesianProductOperation<R2_t, SO2_t> R2crossSO2_t;
    
    typedef Eigen::Matrix<Scalar,2,2,Options> Matrix2;
    typedef Eigen::Matrix<Scalar,2,1,Options> Vector2;

    template<typename TangentVector, typename Matrix2Like, typename Vector2Like>
    static void exp(const Eigen::MatrixBase<TangentVector> & v,
                    const Eigen::MatrixBase<Matrix2Like> & R,
                    const Eigen::MatrixBase<Vector2Like> & t)
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like, 2, 2);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector2Like, 2);

      typedef typename Matrix2Like::Scalar Scalar;
      const Scalar omega = v(2);
      Scalar cv,sv; SINCOS(omega, &sv, &cv);
      PINOCCHIO_EIGEN_CONST_CAST(Matrix2Like,R) << cv, -sv, sv, cv;
      using internal::if_then_else;

      {
        typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector2Like) vcross(-v(1), v(0));
        vcross -= -v(1)*R.col(0) + v(0)*R.col(1);
        vcross /= omega;
        Scalar omega_abs = math::fabs(omega);
        PINOCCHIO_EIGEN_CONST_CAST(Vector2Like,t).coeffRef(0) = if_then_else(internal::GT, omega_abs , Scalar(1e-14),
                                                                             vcross.coeff(0),
                                                                             v.coeff(0));
        
        PINOCCHIO_EIGEN_CONST_CAST(Vector2Like,t).coeffRef(1) = if_then_else(internal::GT, omega_abs, Scalar(1e-14),
                                                                             vcross.coeff(1),
                                                                             v.coeff(1));
      }
      
    }

    template<typename Matrix2Like, typename Vector2Like, typename Matrix3Like>
    static void toInverseActionMatrix(const Eigen::MatrixBase<Matrix2Like> & R,
                                      const Eigen::MatrixBase<Vector2Like> & t,
                                      const Eigen::MatrixBase<Matrix3Like> & M,
                                      const AssignmentOperatorType op)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like, 2, 2);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector2Like, 2);
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like, M, 3, 3);
      Matrix3Like & Mout = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,M);
      typedef typename Matrix3Like::Scalar Scalar;
      
      typename PINOCCHIO_EIGEN_PLAIN_TYPE(Vector2Like) tinv((R.transpose() * t).reverse());
      tinv[0] *= Scalar(-1.);
      switch(op)
        {
        case SETTO:
          Mout.template topLeftCorner<2,2>() = R.transpose();
          Mout.template topRightCorner<2,1>() = tinv;
          Mout.template bottomLeftCorner<1,2>().setZero();
          Mout(2,2) = (Scalar)1;
          break;
        case ADDTO:
          Mout.template topLeftCorner<2,2>() += R.transpose();
          Mout.template topRightCorner<2,1>() += tinv;
          Mout(2,2) += (Scalar)1;
          break;
        case RMTO:
          Mout.template topLeftCorner<2,2>() -= R.transpose();
          Mout.template topRightCorner<2,1>() -= tinv;
          Mout(2,2) -= (Scalar)1;
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
        }


      
    }

    template<typename Matrix2Like, typename Vector2Like, typename TangentVector>
    static void log(const Eigen::MatrixBase<Matrix2Like> & R,
                    const Eigen::MatrixBase<Vector2Like> & p,
                    const Eigen::MatrixBase<TangentVector> & v)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like, 2, 2);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector2Like, 2);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      
      TangentVector & vout = PINOCCHIO_EIGEN_CONST_CAST(TangentVector,v);

      typedef typename Matrix2Like::Scalar Scalar1;
      
      Scalar1 t = SO2_t::log(R);
      const Scalar1 tabs = math::fabs(t);
      const Scalar1 t2 = t*t;
      Scalar1 st,ct; SINCOS(tabs, &st, &ct);
      Scalar1 alpha;
      alpha = internal::if_then_else(internal::LT, tabs, Scalar(1e-4),
                                     1 - t2/12 - t2*t2/720,
                                     tabs*st/(2*(1-ct)));

      vout.template head<2>().noalias() = alpha * p;
      vout(0) += t/2 * p(1);
      vout(1) += -t/2 * p(0);
      vout(2) = t;
    }

    template<typename Matrix2Like, typename Vector2Like, typename JacobianOutLike>
    static void Jlog(const Eigen::MatrixBase<Matrix2Like> & R,
                     const Eigen::MatrixBase<Vector2Like> & p,
                     const Eigen::MatrixBase<JacobianOutLike> & J)
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix2Like, 2, 2);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector2Like, 2);
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(JacobianOutLike, JacobianMatrix_t);
      
      JacobianOutLike & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOutLike,J);

      typedef typename Matrix2Like::Scalar Scalar1;
      
      Scalar1 t = SO2_t::log(R);
      const Scalar1 tabs = math::fabs(t);
      Scalar1 alpha, alpha_dot;
      Scalar1 t2 = t*t;
      Scalar1 st,ct; SINCOS(t, &st, &ct);
      Scalar1 inv_2_1_ct = 0.5 / (1-ct);
        
      alpha = internal::if_then_else(internal::LT, tabs, Scalar(1e-4),
                                     1 - t2/12,
                                     t*st*inv_2_1_ct);
      alpha_dot = internal::if_then_else(internal::LT, tabs, Scalar(1e-4),
                                         - t / 6 - t2*t / 180,
                                         (st-t) * inv_2_1_ct);

      typename PINOCCHIO_EIGEN_PLAIN_TYPE(Matrix2Like) V;
      V(0,0) = V(1,1) = alpha;
      V(1,0) = - t / 2;
      V(0,1) = - V(1,0);

      Jout.template topLeftCorner <2,2>().noalias() = V * R;
      Jout.template topRightCorner<2,1>() << alpha_dot*p[0] + p[1]/2, -p(0)/2 + alpha_dot*p(1);
      Jout.template bottomLeftCorner<1,2>().setZero();
      Jout(2,2) = 1;
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
      ConfigVector_t n; n << Scalar(0), Scalar(0), Scalar(1), Scalar(0);
      return n;
    }

    static std::string name()
    {
      return std::string("SE(2)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      Matrix2 R0, R1; Vector2 t0, t1;
      forwardKinematics(R0, t0, q0);
      forwardKinematics(R1, t1, q1);
      Matrix2 R (R0.transpose() * R1);
      Vector2 t (R0.transpose() * (t1 - t0));

      log(R, t, d);
    }

    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                          const Eigen::MatrixBase<ConfigR_t> & q1,
                          const Eigen::MatrixBase<JacobianOut_t>& J) const
    {
      Matrix2 R0, R1; Vector2 t0, t1;
      forwardKinematics(R0, t0, q0);
      forwardKinematics(R1, t1, q1);
      Matrix2 R (R0.transpose() * R1);
      Vector2 t (R0.transpose() * (t1 - t0));

      if (arg == ARG0) {
        JacobianMatrix_t J1;
        Jlog (R, t, J1);

        // pcross = [ y1-y0, - (x1 - x0) ]
        Vector2 pcross (q1(1) - q0(1), q0(0) - q1(0));

        JacobianOut_t& J0 = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t, J);
        J0.template topLeftCorner <2,2> ().noalias() = - R.transpose();
        J0.template topRightCorner<2,1> ().noalias() = R1.transpose() * pcross;
        J0.template bottomLeftCorner <1,2> ().setZero();
        J0 (2,2) = -1;
        J0.applyOnTheLeft(J1);
      } else if (arg == ARG1) {
        Jlog (R, t, J);
      }
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t & out = PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout);

      Matrix2 R0, R;
      Vector2 t0, t;
      forwardKinematics(R0, t0, q);
      exp(v, R, t);

      out.template head<2>().noalias() = R0 * t + t0;
      out.template tail<2>().noalias() = R0 * R.col(0);
    }
    
    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      Jout.setZero();
      
      const typename Config_t::Scalar & c_theta = q(2),
                                      & s_theta = q(3);
      
      Jout.template topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      Jout.template bottomRightCorner<2,1>() << -s_theta, c_theta;
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & v,
                                   const Eigen::MatrixBase<JacobianOut_t>& J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);

      Matrix2 R;
      Vector2 t;
      exp(v, R, t);

      toInverseActionMatrix(R, t, Jout, op);
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t> & v,
                                   const Eigen::MatrixBase<JacobianOut_t> & J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
      // TODO sparse version
      MotionTpl<Scalar,0> nu; nu.toVector() << v.template head<2>(), 0, 0, 0, v[2]; 
      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jexp6(nu, Jtmp6);

      switch(op)
        {
        case SETTO:
          Jout << Jtmp6.template topLeftCorner<2,2>(), Jtmp6.template topRightCorner<2,1>(),
            Jtmp6.template bottomLeftCorner<1,2>(), Jtmp6.template bottomRightCorner<1,1>();
          break;
        case ADDTO:
          Jout.template topLeftCorner<2,2>() += Jtmp6.template topLeftCorner<2,2>();
          Jout.template topRightCorner<2,1>() += Jtmp6.template topRightCorner<2,1>();
          Jout.template bottomLeftCorner<1,2>() += Jtmp6.template bottomLeftCorner<1,2>();
          Jout.template bottomRightCorner<1,1>() += Jtmp6.template bottomRightCorner<1,1>();
          break;
        case RMTO:
          Jout.template topLeftCorner<2,2>() -= Jtmp6.template topLeftCorner<2,2>();
          Jout.template topRightCorner<2,1>() -= Jtmp6.template topRightCorner<2,1>();
          Jout.template bottomLeftCorner<1,2>() -= Jtmp6.template bottomLeftCorner<1,2>();
          Jout.template bottomRightCorner<1,1>() -= Jtmp6.template bottomRightCorner<1,1>();
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
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      Matrix2 R;
      Vector2 t;
      exp(v, R, t);

      Vector2 tinv = (R.transpose() * t).reverse();
      tinv[0] *= Scalar(-1.);

      Jout.template topRows<2>().noalias() = R.transpose() * Jin.template topRows<2>();
      Jout.template topRows<2>().noalias() += tinv * Jin.template bottomRows<1>();
      Jout.template bottomRows<1>() = Jin.template bottomRows<1>();
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      MotionTpl<Scalar,0> nu; nu.toVector() << v.template head<2>(), 0, 0, 0, v[2]; 

      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jexp6(nu, Jtmp6);
      
      Jout.template topRows<2>().noalias()
      = Jtmp6.template topLeftCorner<2,2>() * Jin.template topRows<2>();
      Jout.template topRows<2>().noalias()
      += Jtmp6.template topRightCorner<2,1>() * Jin.template bottomRows<1>();
      Jout.template bottomRows<1>().noalias()
      = Jtmp6.template bottomLeftCorner<1,2>()* Jin.template topRows<2>();
      Jout.template bottomRows<1>().noalias()
      += Jtmp6.template bottomRightCorner<1,1>() * Jin.template bottomRows<1>();

    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J) const
    {
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      Matrix2 R;
      Vector2 t;
      exp(v, R, t);

      Vector2 tinv = (R.transpose() * t).reverse();
      tinv[0] *= Scalar(-1);
      //TODO: Aliasing
      Jout.template topRows<2>() = R.transpose() * Jout.template topRows<2>();
      //No Aliasing
      Jout.template topRows<2>().noalias() += tinv * Jout.template bottomRows<1>();
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J) const
    {
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      MotionTpl<Scalar,0> nu; nu.toVector() << v.template head<2>(), 0, 0, 0, v[2]; 

      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jexp6(nu, Jtmp6);
      //TODO: Remove aliasing
      Jout.template topRows<2>()
      = Jtmp6.template topLeftCorner<2,2>() * Jout.template topRows<2>();
      Jout.template topRows<2>().noalias()
      += Jtmp6.template topRightCorner<2,1>() * Jout.template bottomRows<1>();
      Jout.template bottomRows<1>()
      = Jtmp6.template bottomRightCorner<1,1>() * Jout.template bottomRows<1>();
      Jout.template bottomRows<1>().noalias()
      += Jtmp6.template bottomLeftCorner<1,2>() * Jout.template topRows<2>();
    }

    template <class Config_t>
    static void normalize_impl(const Eigen::MatrixBase<Config_t> & qout)
    {
      PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout).template tail<2>().normalize();
    }

    template <class Config_t>
    static bool isNormalized_impl(const Eigen::MatrixBase<Config_t> & qin,
                                  const Scalar& prec)
    {
      const Scalar norm = Scalar(qin.template tail<2>().norm());
      using std::abs;
      return abs(norm - Scalar(1.0)) < prec;
    }

    template <class Config_t>
    void random_impl(const Eigen::MatrixBase<Config_t> & qout) const
    {
      R2crossSO2_t().random(qout);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      R2crossSO2_t ().randomConfiguration(lower, upper, qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      return R2crossSO2_t().isSameConfiguration(q0, q1, prec);
    }

  protected:
    
    template<typename Matrix2Like, typename Vector2Like, typename Vector4Like>
    static void forwardKinematics(const Eigen::MatrixBase<Matrix2Like> & R,
                                  const Eigen::MatrixBase<Vector2Like> & t,
                                  const Eigen::MatrixBase<Vector4Like> & q)
    {
      EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix2Like, Matrix2);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Vector2Like, Vector2);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,Vector4Like);

      PINOCCHIO_EIGEN_CONST_CAST(Vector2Like,t) = q.template head<2>();
      const typename Vector4Like::Scalar & c_theta = q(2),
                                         & s_theta = q(3);
      PINOCCHIO_EIGEN_CONST_CAST(Matrix2Like,R) << c_theta, -s_theta, s_theta, c_theta;
      
    }
  }; // struct SpecialEuclideanOperationTpl<2>

  template<typename _Scalar, int _Options>
  struct traits< SpecialEuclideanOperationTpl<3,_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options,
      NQ = 7,
      NV = 6
    };
  };
  
  /// SE(3)
  template<typename _Scalar, int _Options>
  struct SpecialEuclideanOperationTpl<3,_Scalar,_Options>
  : public LieGroupBase <SpecialEuclideanOperationTpl<3,_Scalar,_Options> >
  {
    PINOCCHIO_LIE_GROUP_TPL_PUBLIC_INTERFACE(SpecialEuclideanOperationTpl);
    
    typedef CartesianProductOperation <VectorSpaceOperationTpl<3,Scalar,Options>, SpecialOrthogonalOperationTpl<3,Scalar,Options> > R3crossSO3_t;

    typedef Eigen::Quaternion<Scalar,Options> Quaternion_t;
    typedef Eigen::Map<      Quaternion_t> QuaternionMap_t;
    typedef Eigen::Map<const Quaternion_t> ConstQuaternionMap_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
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
      ConfigVector_t n; n.template head<6>().setZero(); n[6] = 1;
      return n;
    }

    static std::string name()
    {
      return std::string("SE(3)");
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      ConstQuaternionMap_t quat0 (q0.derived().template tail<4>().data());
      assert(quaternion::isNormalized(quat0,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat1 (q1.derived().template tail<4>().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      PINOCCHIO_EIGEN_CONST_CAST(Tangent_t,d)
        = log6(  SE3(quat0.matrix(), q0.derived().template head<3>()).inverse()
               * SE3(quat1.matrix(), q1.derived().template head<3>())).toVector();
    }

    /// \cheatsheet \f$ \frac{\partial\ominus}{\partial q_1} {}^1X_0 = - \frac{\partial\ominus}{\partial q_0} \f$
    template <ArgumentPosition arg, class ConfigL_t, class ConfigR_t, class JacobianOut_t>
    void dDifference_impl (const Eigen::MatrixBase<ConfigL_t> & q0,
                           const Eigen::MatrixBase<ConfigR_t> & q1,
                           const Eigen::MatrixBase<JacobianOut_t> & J) const
    {
      typedef typename SE3::Vector3 Vector3;
      typedef typename SE3::Matrix3 Matrix3;

      ConstQuaternionMap_t quat0 (q0.derived().template tail<4>().data());
      assert(quaternion::isNormalized(quat0,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      ConstQuaternionMap_t quat1 (q1.derived().template tail<4>().data());
      assert(quaternion::isNormalized(quat1,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      
      Matrix3 R0(quat0.matrix()), R1 (quat1.matrix());
      assert(isUnitary(R0)); assert(isUnitary(R1));
      
      const SE3 M (  SE3(R0, q0.template head<3>()).inverse()
                   * SE3(R1, q1.template head<3>()));

      if (arg == ARG0) {
        JacobianMatrix_t J1;
        Jlog6 (M, J1);

        const Vector3 p1_p0 = R1.transpose()*(q1.template head<3>() - q0.template head<3>());

        JacobianOut_t & J0 = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);
        J0.template bottomRightCorner<3,3> ().noalias() = J0.template topLeftCorner <3,3> ().noalias() = - M.rotation().transpose();
        J0.template topRightCorner<3,3> ().noalias() = skew(p1_p0) * M.rotation().transpose(); // = R1.T * skew(q1_t - q0_t) * R0;
        J0.template bottomLeftCorner<3,3> ().setZero();
        J0.applyOnTheLeft(J1);
      }
      else if (arg == ARG1) {
        Jlog6 (M, J);
      }
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      ConfigOut_t & out = PINOCCHIO_EIGEN_CONST_CAST(ConfigOut_t,qout);
      Quaternion_t const quat(q.derived().template tail<4>());
      assert(quaternion::isNormalized(quat,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      QuaternionMap_t res_quat (out.template tail<4>().data());
      
      using internal::if_then_else;

      SE3 M0 (quat.matrix(), q.derived().template head<3>());
      MotionRef<const Velocity_t> mref_v(v.derived());
      SE3 M1 (M0 * exp6(mref_v));

      out.template head<3>() = M1.translation();
      quaternion::assignQuaternion(res_quat,M1.rotation()); // required by CasADi
      const Scalar dot_product = res_quat.dot(quat);
      for(Eigen::DenseIndex k = 0; k < 4; ++k)
      {
        res_quat.coeffs().coeffRef(k) = if_then_else(internal::LT, dot_product, Scalar(0),
                                                     -res_quat.coeffs().coeff(k),
                                                      res_quat.coeffs().coeff(k));
      }
    
      // Norm of qs might be epsilon-different to 1, so M1.rotation might be epsilon-different to a rotation matrix.
      // It is then safer to re-normalized after converting M1.rotation to quaternion.
      quaternion::firstOrderNormalize(res_quat);
      assert(quaternion::isNormalized(res_quat,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
    }
    
    template <class Config_t, class Jacobian_t>
    static void integrateCoeffWiseJacobian_impl(const Eigen::MatrixBase<Config_t> & q,
                                                const Eigen::MatrixBase<Jacobian_t> & J)
    {
      assert(J.rows() == nq() && J.cols() == nv() && "J is not of the right dimension");
      
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Config_t) ConfigPlainType;
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Jacobian_t) JacobianPlainType;
      typedef typename ConfigPlainType::Scalar Scalar;

      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J);
      Jout.setZero();
      
      ConstQuaternionMap_t quat_map(q.derived().template tail<4>().data());
      assert(quaternion::isNormalized(quat_map,RealScalar(PINOCCHIO_DEFAULT_QUATERNION_NORM_TOLERANCE_VALUE)));
      Jout.template topLeftCorner<3,3>() = quat_map.toRotationMatrix();
//      Jexp3(quat,Jout.template bottomRightCorner<4,3>());
      
      typedef Eigen::Matrix<Scalar,4,3,JacobianPlainType::Options|Eigen::RowMajor> Jacobian43;
      typedef SE3Tpl<Scalar,ConfigPlainType::Options> SE3;
      Jacobian43 Jexp3QuatCoeffWise;
      
      Scalar theta;
      typename SE3::Vector3 v = quaternion::log3(quat_map,theta);
      quaternion::Jexp3CoeffWise(v,Jexp3QuatCoeffWise);
      typename SE3::Matrix3 Jlog;
      Jlog3(theta,v,Jlog);
      
//      std::cout << "Jexp3QuatCoeffWise\n" << Jexp3QuatCoeffWise << std::endl;
//      std::cout << "Jlog\n" << Jlog << std::endl;
      
//      if(quat_map.w() >= 0.) // comes from the log3 for quaternions which may change the sign.
      if(quat_map.coeffs()[3] >= Scalar(0)) // comes from the log3 for quaternions which may change the sign.
        PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).template bottomRightCorner<4,3>().noalias() = Jexp3QuatCoeffWise * Jlog;
      else
        PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J).template bottomRightCorner<4,3>().noalias() = -Jexp3QuatCoeffWise * Jlog;
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dq_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & v,
                                   const Eigen::MatrixBase<JacobianOut_t>& J,
                                   const AssignmentOperatorType op=SETTO)
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J);

      switch(op)
      {
        case SETTO:
          Jout = exp6(MotionRef<const Tangent_t>(v.derived())).toDualActionMatrix().transpose();
          break;
        case ADDTO:
          Jout += exp6(MotionRef<const Tangent_t>(v.derived())).toDualActionMatrix().transpose();
          break;
        case RMTO:
          Jout -= exp6(MotionRef<const Tangent_t>(v.derived())).toDualActionMatrix().transpose();
          break;
        default:
          assert(false && "Wrong Op requesed value");
          break;
      }
    }

    template <class Config_t, class Tangent_t, class JacobianOut_t>
    static void dIntegrate_dv_impl(const Eigen::MatrixBase<Config_t >  & /*q*/,
                                   const Eigen::MatrixBase<Tangent_t>  & v,
                                   const Eigen::MatrixBase<JacobianOut_t>& J,
                                   const AssignmentOperatorType op=SETTO)
    {
      switch(op)
      {
        case SETTO:
          Jexp6<SETTO>(MotionRef<const Tangent_t>(v.derived()), J.derived());
          break;
        case ADDTO:
          Jexp6<ADDTO>(MotionRef<const Tangent_t>(v.derived()), J.derived());
          break;
        case RMTO:
          Jexp6<RMTO>(MotionRef<const Tangent_t>(v.derived()), J.derived());
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
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jtmp6 = exp6(MotionRef<const Tangent_t>(v.derived())).toDualActionMatrix().transpose();
      
      Jout.template topRows<3>().noalias()
      = Jtmp6.template topLeftCorner<3,3>() * Jin.template topRows<3>();
      Jout.template topRows<3>().noalias()
      += Jtmp6.template topRightCorner<3,3>() * Jin.template bottomRows<3>();
      Jout.template bottomRows<3>().noalias()
      = Jtmp6.template bottomRightCorner<3,3>() * Jin.template bottomRows<3>();
    }

    template <class Config_t, class Tangent_t, class JacobianIn_t, class JacobianOut_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<JacobianIn_t> & Jin,
                                     const Eigen::MatrixBase<JacobianOut_t> & J_out) const
    {
      JacobianOut_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(JacobianOut_t,J_out);
      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jexp6<SETTO>(MotionRef<const Tangent_t>(v.derived()), Jtmp6);

      Jout.template topRows<3>().noalias()
      = Jtmp6.template topLeftCorner<3,3>() * Jin.template topRows<3>();
      Jout.template topRows<3>().noalias()
      += Jtmp6.template topRightCorner<3,3>() * Jin.template bottomRows<3>();
      Jout.template bottomRows<3>().noalias()
      = Jtmp6.template bottomRightCorner<3,3>() * Jin.template bottomRows<3>();
    }


    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dq_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J_out) const
    {
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J_out);
      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jtmp6 = exp6(MotionRef<const Tangent_t>(v.derived())).toDualActionMatrix().transpose();

      //Aliasing
      Jout.template topRows<3>()
      = Jtmp6.template topLeftCorner<3,3>() * Jout.template topRows<3>();
      Jout.template topRows<3>().noalias()
      += Jtmp6.template topRightCorner<3,3>() * Jout.template bottomRows<3>();
      Jout.template bottomRows<3>()
      = Jtmp6.template bottomRightCorner<3,3>() * Jout.template bottomRows<3>();
    }

    template <class Config_t, class Tangent_t, class Jacobian_t>
    void dIntegrateTransport_dv_impl(const Eigen::MatrixBase<Config_t > & /*q*/,
                                     const Eigen::MatrixBase<Tangent_t> & v,
                                     const Eigen::MatrixBase<Jacobian_t> & J_out) const
    {
      Jacobian_t & Jout = PINOCCHIO_EIGEN_CONST_CAST(Jacobian_t,J_out);
      Eigen::Matrix<Scalar,6,6> Jtmp6;
      Jexp6<SETTO>(MotionRef<const Tangent_t>(v.derived()), Jtmp6);

      Jout.template topRows<3>()
      = Jtmp6.template topLeftCorner<3,3>() * Jout.template topRows<3>();
      Jout.template topRows<3>().noalias()
      += Jtmp6.template topRightCorner<3,3>() * Jout.template bottomRows<3>();
      Jout.template bottomRows<3>()
      = Jtmp6.template bottomRightCorner<3,3>() * Jout.template bottomRows<3>();
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
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& qout)
    {
      Config_t & qout_ = PINOCCHIO_EIGEN_CONST_CAST(Config_t,qout);
      qout_.template tail<4>().normalize();
    }

    template <class Config_t>
    static bool isNormalized_impl(const Eigen::MatrixBase<Config_t>& qin,
                                  const Scalar& prec)
    {
      Scalar norm = Scalar(qin.template tail<4>().norm());
      using std::abs;
      return abs(norm - Scalar(1.0)) < prec;
    }

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      R3crossSO3_t().random(qout);
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & lower,
                                  const Eigen::MatrixBase<ConfigR_t> & upper,
                                  const Eigen::MatrixBase<ConfigOut_t> & qout)
      const
    {
      R3crossSO3_t ().randomConfiguration(lower, upper, qout);
    }

    template <class ConfigL_t, class ConfigR_t>
    static bool isSameConfiguration_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                         const Eigen::MatrixBase<ConfigR_t> & q1,
                                         const Scalar & prec)
    {
      return R3crossSO3_t().isSameConfiguration(q0, q1, prec);
    }
  }; // struct SpecialEuclideanOperationTpl<3>

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_liegroup_special_euclidean_operation_hpp__
