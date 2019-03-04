//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_planar_hpp__
#define __pinocchio_joint_planar_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/spatial/cartesian-axis.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace pinocchio
{
  
  template<typename Scalar, int Options = 0> struct MotionPlanarTpl;
  typedef MotionPlanarTpl<double> MotionPlanar;
  
  namespace internal
  {
    template<typename Scalar, int Options>
    struct SE3GroupAction< MotionPlanarTpl<Scalar,Options> >
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< MotionPlanarTpl<Scalar,Options>, MotionDerived>
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
  }
  
  template<typename _Scalar, int _Options>
  struct traits< MotionPlanarTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionPlanarTpl

  template<typename _Scalar, int _Options>
  struct MotionPlanarTpl : MotionBase< MotionPlanarTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionPlanarTpl);
    
    typedef CartesianAxis<2> AxisZ;

    MotionPlanarTpl() {}
    
    MotionPlanarTpl(const Scalar & x_dot, const Scalar & y_dot,
                    const Scalar & theta_dot)
    : m_x_dot(x_dot), m_y_dot(y_dot), m_theta_dot(theta_dot)
    {}
    
    template<typename Vector3Like>
    MotionPlanarTpl(const Eigen::MatrixBase<Vector3Like> & vj)
    : m_x_dot(vj[0]), m_y_dot(vj[1]), m_theta_dot(vj[2])
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
    }

//    operator MotionPlain() const
//    {
//      return MotionPlain(typename MotionPlain::Vector3(m_x_dot,m_y_dot,Scalar(0)),
//                         typename MotionPlain::Vector3(Scalar(0),Scalar(0),m_theta_dot));
//    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & other) const
    {
      other.linear()[0] += m_x_dot;
      other.linear()[1] += m_y_dot;
      other.angular()[2] += m_theta_dot;
    }
    
    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & other) const
    {
      other.linear()  <<   m_x_dot,   m_y_dot,   (Scalar)0;
      other.angular() << (Scalar)0, (Scalar)0, m_theta_dot;
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().noalias() = m.rotation().col(2) * m_theta_dot;
      v.linear().noalias() = m.translation().cross(v.angular());
      v.linear() += m.rotation().col(0) * m_x_dot;
      v.linear() += m.rotation().col(1) * m_y_dot;
    }
    
    template<typename S2, int O2>
    MotionPlain se3Action_impl(const SE3Tpl<S2,O2> & m) const
    {
      MotionPlain res;
      se3Action_impl(m,res);
      return res;
    }
    
    template<typename S2, int O2, typename D2>
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      // Linear
      // TODO: use v.angular() as temporary variable
      Vector3 v3_tmp;
      AxisZ::alphaCross(m_theta_dot,m.translation(),v3_tmp);
      v3_tmp[0] += m_x_dot; v3_tmp[1] += m_y_dot;
      v.linear().noalias() = m.rotation().transpose() * v3_tmp;
      
      // Angular
      v.angular().noalias() = m.rotation().transpose().col(2) * m_theta_dot;
    }
    
    template<typename S2, int O2>
    MotionPlain se3ActionInverse_impl(const SE3Tpl<S2,O2> & m) const
    {
      MotionPlain res;
      se3ActionInverse_impl(m,res);
      return res;
    }
    
    template<typename M1, typename M2>
    void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
    {
      // Linear
      AxisZ::alphaCross(-m_theta_dot,v.linear(),mout.linear());
      
      typename M1::ConstAngularType w_in = v.angular();
      typename M2::LinearType v_out = mout.linear();
      
      v_out[0] -= w_in[2] * m_y_dot;
      v_out[1] += w_in[2] * m_x_dot;
      v_out[2] += -w_in[1] * m_x_dot + w_in[0] * m_y_dot ;
      
      // Angular
      AxisZ::alphaCross(-m_theta_dot,v.angular(),mout.angular());
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }

    // data
    Scalar m_x_dot;
    Scalar m_y_dot;
    Scalar m_theta_dot;
    
  }; // struct MotionPlanarTpl
  
  template<typename Scalar, int Options, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionPlanarTpl<Scalar,Options> & m1, const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain result(m2);
    result.linear()[0] += m1.m_x_dot;
    result.linear()[1] += m1.m_y_dot;

    result.angular()[2] += m1.m_theta_dot;

    return result;
  }

  template<typename Scalar, int Options> struct ConstraintPlanarTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintPlanarTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,4,1,Options> Vector4;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<Scalar,Options> Quaternion_t;
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef ForceTpl<Scalar,Options> Force;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionPlanarTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,3,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,3,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // struct traits ConstraintPlanarTpl

  template<typename _Scalar, int _Options>
  struct ConstraintPlanarTpl : ConstraintBase< ConstraintPlanarTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintPlanarTpl);
    
    enum { NV = 3, Options = _Options }; // to check
    typedef typename traits<ConstraintPlanarTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintPlanarTpl>::JointForce JointForce;
    typedef typename traits<ConstraintPlanarTpl>::DenseBase DenseBase;

    template<typename Vector3Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector3Like> & vj) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return JointMotion(vj);
    }

    int nv_impl() const { return NV; }

    struct ConstraintTranspose
    {
      const ConstraintPlanarTpl & ref;
      ConstraintTranspose(const ConstraintPlanarTpl & ref) : ref(ref) {}

      template<typename Derived>
      typename ForceDense<Derived>::Vector3 operator* (const ForceDense<Derived> & phi)
      {
        typedef typename ForceDense<Derived>::Vector3 Vector3;
        return Vector3(phi.linear()[0], phi.linear()[1], phi.angular()[2]);
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename Derived>
      friend typename Eigen::Matrix <typename Eigen::MatrixBase<Derived>::Scalar, 3, Derived::ColsAtCompileTime>
      operator*(const ConstraintTranspose &, const Eigen::MatrixBase<Derived> & F)
      {
        typedef typename Eigen::MatrixBase<Derived>::Scalar Scalar;
        typedef Eigen::Matrix<Scalar, 3, Derived::ColsAtCompileTime> MatrixReturnType;
        assert(F.rows()==6);

        MatrixReturnType result(3, F.cols ());
        result.template topRows<2>() = F.template topRows<2>();
        result.template bottomRows<1>() = F.template bottomRows<1>();
        return result;
      }
      
    }; // struct ConstraintTranspose

    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template block<3,3>(Inertia::LINEAR, 0).setZero ();
      S.template block<3,3>(Inertia::ANGULAR, 0).setZero ();
      S(Inertia::LINEAR + 0,0) = Scalar(1);
      S(Inertia::LINEAR + 1,1) = Scalar(1);
      S(Inertia::ANGULAR + 2,2) = Scalar(1);
      return S;
    }

    template<typename S1, int O1>
    DenseBase se3Action(const SE3Tpl<S1,O1> & m) const
    {
      DenseBase X_subspace;
      X_subspace.template block <3,2>(Motion::LINEAR, 0) = m.rotation ().template leftCols <2> ();
      X_subspace.template block <3,1>(Motion::LINEAR, 2) = m.translation().cross(m.rotation ().template rightCols<1>());

      X_subspace.template block <3,2>(Motion::ANGULAR, 0).setZero ();
      X_subspace.template block <3,1>(Motion::ANGULAR, 2) = m.rotation ().template rightCols<1>();

      return X_subspace;
    }

    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();
      DenseBase res(DenseBase::Zero());
      
      res(0,1) = -w[2]; res(0,2) = v[1];
      res(1,0) = w[2]; res(1,2) = -v[0];
      res(2,0) = -w[1]; res(2,1) = w[0];
      res(3,2) = w[1];
      res(4,2) = -w[0];
      
      return res;
    }
  }; // struct ConstraintPlanarTpl

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionPlanarTpl<S2,O2> & m2)
  {
    return m2.motionAction(m1);
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline typename Eigen::Matrix<S1,6,3,O1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPlanarTpl<S2,O2> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    typedef typename Inertia::Scalar Scalar;
    typedef Eigen::Matrix<S1,6,3,O1> ReturnType;
    
    ReturnType M;
    const Scalar mass = Y.mass();
    const typename Inertia::Vector3 & com = Y.lever();
    const typename Inertia::Symmetric3 & inertia = Y.inertia();

    M.template topLeftCorner<3,3>().setZero();
    M.template topLeftCorner<2,2>().diagonal().fill(mass);

    const typename Inertia::Vector3 mc(mass * com);
    M.template rightCols<1>().template head<2>() << -mc(1), mc(0);

    M.template bottomLeftCorner<3,2>() << (Scalar)0, -mc(2), mc(2), (Scalar)0, -mc(1), mc(0);
    M.template rightCols<1>().template tail<3>() = inertia.data().template tail<3>();
    M.template rightCols<1>()[3] -= mc(0)*com(2);
    M.template rightCols<1>()[4] -= mc(1)*com(2);
    M.template rightCols<1>()[5] += mass*(com(0)*com(0) + com(1)*com(1));

    return M;
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  //  inline Eigen::Matrix<double,6,1>
  
  template<typename M6Like, typename S2, int O2>
  inline Eigen::Matrix<S2,6,3,O2>
  operator*(const Eigen::MatrixBase<M6Like> & Y,
            const ConstraintPlanarTpl<S2,O2> &)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
    typedef Eigen::Matrix<S2,6,3,O2> Matrix63;
    
    Matrix63 IS;
    IS.template leftCols<2>() = Y.template leftCols<2>();
    IS.template rightCols<1>() = Y.template rightCols<1>();
    
    return IS;
  }
  
  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintPlanarTpl<S1,O1> >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintPlanarTpl<S1,O1>,MotionDerived >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  }

  template<typename Scalar, int Options> struct JointPlanarTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointPlanarTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 4,
      NV = 3
    };
    enum { Options = _Options };
    typedef _Scalar Scalar;
    typedef JointDataPlanarTpl<Scalar,Options> JointDataDerived;
    typedef JointModelPlanarTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintPlanarTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionPlanarTpl<Scalar,Options> Motion_t;
    typedef BiasZeroTpl<Scalar,Options> Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options>
  struct traits< JointDataPlanarTpl<Scalar,Options> > { typedef JointPlanarTpl<Scalar,Options> JointDerived; };
  template<typename Scalar, int Options>
  struct traits< JointModelPlanarTpl<Scalar,Options> > { typedef JointPlanarTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataPlanarTpl : public JointDataBase< JointDataPlanarTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPlanarTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    
    D_t StU;

    JointDataPlanarTpl () : M(1), U(), Dinv(), UDinv() {}

    static std::string classname() { return std::string("JointDataPlanar"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataPlanarTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelPlanarTpl);
  template<typename _Scalar, int _Options>
  struct JointModelPlanarTpl
  : public JointModelBase< JointModelPlanarTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPlanarTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE;
    
    typedef JointModelBase<JointModelPlanarTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<ConfigVector> & q_joint) const
    {
      const Scalar
      & c_theta = q_joint(2),
      & s_theta = q_joint(3);
      
      M.rotation().template topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      M.translation().template head<2>() = q_joint.template head<2>();
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar Scalar;
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type & q = qs.template segment<NQ>(idx_q());

      const Scalar
      &c_theta = q(2),
      &s_theta = q(3);

      data.M.rotation().template topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation().template head<2>() = q.template head<2>();

    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      typename TangentVector::template ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.template segment<NV>(idx_v ());

      data.v.m_x_dot = q_dot(0);
      data.v.m_y_dot = q_dot(1);
      data.v.m_theta_dot = q_dot(2);
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    {
      data.U.template leftCols<2>() = I.template leftCols<2>();
      data.U.template rightCols<1>() = I.template rightCols<1>();

      data.StU.template leftCols<2>() = data.U.template topRows<2>().transpose();
      data.StU.template rightCols<1>() = data.U.template bottomRows<1>();
      
      // compute inverse
      data.Dinv.setIdentity();
      data.StU.llt().solveInPlace(data.Dinv);
      
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using math::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelPlanar");}
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelPlanarTpl<NewScalar,Options> cast() const
    {
      typedef JointModelPlanarTpl<NewScalar,Options> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelPlanarTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelPlanarTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelPlanarTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataPlanarTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataPlanarTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_planar_hpp__
