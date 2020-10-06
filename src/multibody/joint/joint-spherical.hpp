//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_spherical_hpp__
#define __pinocchio_joint_spherical_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options = 0> struct MotionSphericalTpl;
  typedef MotionSphericalTpl<double> MotionSpherical;
  
  template<typename Scalar, int Options>
  struct SE3GroupAction< MotionSphericalTpl<Scalar,Options> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction< MotionSphericalTpl<Scalar,Options>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits< MotionSphericalTpl<_Scalar,_Options> >
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
    typedef MotionPlain PlainReturnType;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionSphericalTpl

  template<typename _Scalar, int _Options>
  struct MotionSphericalTpl
  : MotionBase< MotionSphericalTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MOTION_TYPEDEF_TPL(MotionSphericalTpl);

    MotionSphericalTpl() {}
    
    template<typename Vector3Like>
    MotionSphericalTpl(const Eigen::MatrixBase<Vector3Like> & w)
    : m_w(w)
    {}

    Vector3 & operator() () { return m_w; }
    const Vector3 & operator() () const { return m_w; }

    inline PlainReturnType plain() const
    {
      return PlainReturnType(PlainReturnType::Vector3::Zero(), m_w);
    }
    
    template<typename MotionDerived>
    void addTo(MotionDense<MotionDerived> & other) const
    {
      other.angular() += m_w;
    }
    
    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear().setZero();
      other.angular() = m_w;
    }
    
    MotionSphericalTpl __plus__(const MotionSphericalTpl & other) const
    {
      return MotionSphericalTpl(m_w + other.m_w);
    }
    
    bool isEqual_impl(const MotionSphericalTpl & other) const
    {
      return m_w == other.m_w;
    }
    
    template<typename MotionDerived>
    bool isEqual_impl(const MotionDense<MotionDerived> & other) const
    {
      return other.angular() == m_w && other.linear().isZero(0);
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      // Angular
      v.angular().noalias() = m.rotation() * m_w;
      
      // Linear
      v.linear().noalias() = m.translation().cross(v.angular());
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
      v3_tmp.noalias() = m_w.cross(m.translation());
      v.linear().noalias() = m.rotation().transpose() * v3_tmp;
      
      // Angular
      v.angular().noalias() = m.rotation().transpose() * m_w;
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
      mout.linear().noalias() = v.linear().cross(m_w);
      
      // Angular
      mout.angular().noalias() = v.angular().cross(m_w);
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    const Vector3 & angular() const { return m_w; }
    Vector3 & angular() { return m_w; }

  protected:
    
    Vector3 m_w;
  }; // struct MotionSphericalTpl

  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionSphericalTpl<S1,O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    return typename MotionDerived::MotionPlain(m2.linear(),m2.angular() + m1.angular());
  }

  template<typename Scalar, int Options> struct ConstraintSphericalTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintSphericalTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionSphericalTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,3,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,3,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // struct traits struct ConstraintSphericalTpl

  template<typename _Scalar, int _Options>
  struct ConstraintSphericalTpl
  : public ConstraintBase< ConstraintSphericalTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintSphericalTpl)
    
    ConstraintSphericalTpl() {}
    
    enum { NV = 3 };
    
    int nv_impl() const { return NV; }
    
    template<typename Vector3Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector3Like> & w) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return JointMotion(w);
    }
    
    struct TransposeConst
    {
      template<typename Derived>
      typename ForceDense<Derived>::ConstAngularType
      operator* (const ForceDense<Derived> & phi)
      {  return phi.angular();  }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename MatrixDerived>
      const typename SizeDepType<3>::RowsReturn<MatrixDerived>::ConstType
      operator*(const Eigen::MatrixBase<MatrixDerived> & F) const
      {
        assert(F.rows()==6);
        return F.derived().template middleRows<3>(Inertia::ANGULAR);
      }
    };

    TransposeConst transpose() const { return TransposeConst(); }

    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template block <3,3>(LINEAR,0).setZero();
      S.template block <3,3>(ANGULAR,0).setIdentity();
      return S;
    }

    template<typename S1, int O1>
    Eigen::Matrix<S1,6,3,O1> se3Action(const SE3Tpl<S1,O1> & m) const
    {
      Eigen::Matrix<S1,6,3,O1> X_subspace;
      cross(m.translation(),m.rotation(),X_subspace.template middleRows<3>(LINEAR));
      X_subspace.template middleRows<3>(ANGULAR) = m.rotation();

      return X_subspace;
    }
    
    template<typename S1, int O1>
    Eigen::Matrix<S1,6,3,O1> se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      Eigen::Matrix<S1,6,3,O1> X_subspace;
      AxisX::cross(m.translation(),X_subspace.template middleRows<3>(ANGULAR).col(0));
      AxisY::cross(m.translation(),X_subspace.template middleRows<3>(ANGULAR).col(1));
      AxisZ::cross(m.translation(),X_subspace.template middleRows<3>(ANGULAR).col(2));
      
      X_subspace.template middleRows<3>(LINEAR).noalias()
      = m.rotation().transpose() * X_subspace.template middleRows<3>(ANGULAR);
      X_subspace.template middleRows<3>(ANGULAR) = m.rotation().transpose();
      
      return X_subspace;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();
      
      DenseBase res;
      skew(v,res.template middleRows<3>(LINEAR));
      skew(w,res.template middleRows<3>(ANGULAR));
      
      return res;
    }
    
    bool isEqual(const ConstraintSphericalTpl &) const { return true; }

  }; // struct ConstraintSphericalTpl

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionSphericalTpl<S2,O2> & m2)
  {
    return m2.motionAction(m1);
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,3,O2>
  operator*(const InertiaTpl<S1,O1> & Y,
            const ConstraintSphericalTpl<S2,O2> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    typedef typename Inertia::Symmetric3 Symmetric3;
    Eigen::Matrix<S2,6,3,O2> M;
    //    M.block <3,3> (Inertia::LINEAR, 0) = - Y.mass () * skew(Y.lever ());
    M.template block<3,3>(Inertia::LINEAR,0) = alphaSkew(-Y.mass(), Y.lever());
    M.template block<3,3>(Inertia::ANGULAR,0) = (Y.inertia() - typename Symmetric3::AlphaSkewSquare(Y.mass(), Y.lever())).matrix();
    return M;
  }

  /* [ABA] Y*S operator*/
  template<typename M6Like, typename S2, int O2>
  inline typename SizeDepType<3>::ColsReturn<M6Like>::ConstType
  operator*(const Eigen::MatrixBase<M6Like> & Y,
            const ConstraintSphericalTpl<S2,O2> &)
  {
    typedef ConstraintSphericalTpl<S2,O2> Constraint;
    return Y.derived().template middleCols<3>(Constraint::ANGULAR);
  }
  
  template<typename S1, int O1>
  struct SE3GroupAction< ConstraintSphericalTpl<S1,O1> >
  { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  
  template<typename S1, int O1, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintSphericalTpl<S1,O1>,MotionDerived >
  { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };

  template<typename Scalar, int Options> struct JointSphericalTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointSphericalTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 4,
      NV = 3
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataSphericalTpl<Scalar,Options> JointDataDerived;
    typedef JointModelSphericalTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintSphericalTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionSphericalTpl<Scalar,Options> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options>
  struct traits< JointDataSphericalTpl<Scalar,Options> >
  { typedef JointSphericalTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits< JointModelSphericalTpl<Scalar,Options> >
  { typedef JointSphericalTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataSphericalTpl : public JointDataBase< JointDataSphericalTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointSphericalTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataSphericalTpl ()
    : M(Transformation_t::Identity())
    , v(Motion_t::Vector3::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}

    static std::string classname() { return std::string("JointDataSpherical"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataSphericalTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelSphericalTpl);
  template<typename _Scalar, int _Options>
  struct JointModelSphericalTpl
  : public JointModelBase< JointModelSphericalTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointSphericalTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelSphericalTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename ConfigVectorLike>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<ConfigVectorLike> & q_joint) const
    {
      typedef typename ConfigVectorLike::Scalar Scalar;
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVectorLike);
      typedef typename Eigen::Quaternion<Scalar,PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorLike)::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      ConstQuaternionMap quat(q_joint.derived().data());
      //assert(math::fabs(quat.coeffs().squaredNorm()-1.) <= sqrt(Eigen::NumTraits<typename V::Scalar>::epsilon())); TODO: check validity of the rhs precision
      assert(math::fabs(static_cast<Scalar>(quat.coeffs().squaredNorm()-1)) <= 1e-4);
      
      M.rotation(quat.matrix());
      M.translation().setZero();
    }
    
    template<typename QuaternionDerived>
    void calc(JointDataDerived & data,
              const typename Eigen::QuaternionBase<QuaternionDerived> & quat) const
    {
      data.M.rotation(quat.matrix());
    }
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::PlainObjectBase<ConfigVector> & qs) const
    {
      typedef typename Eigen::Quaternion<typename ConfigVector::Scalar,ConfigVector::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      ConstQuaternionMap quat(qs.template segment<NQ>(idx_q()).data());
      calc(data,quat);
    }

    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename Eigen::Quaternion<Scalar,Options> Quaternion;

      const Quaternion quat(qs.template segment<NQ>(idx_q()));
      calc(data,quat);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      data.v.angular() = vs.template segment<NV>(idx_v());
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U = I.template block<6,3>(0,Inertia::ANGULAR);
      
      // compute inverse
//      data.Dinv.setIdentity();
//      data.U.template middleRows<3>(Inertia::ANGULAR).llt().solveInPlace(data.Dinv);
      internal::PerformStYSInversion<Scalar>::run(data.U.template middleRows<3>(Inertia::ANGULAR),data.Dinv);
      
      data.UDinv.template middleRows<3>(Inertia::ANGULAR).setIdentity(); // can be put in data constructor
      data.UDinv.template middleRows<3>(Inertia::LINEAR).noalias() = data.U.template block<3,3>(Inertia::LINEAR, 0) * data.Dinv;
      
      if (update_I)
      {
        Matrix6Like & I_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I);
        I_.template block<3,3>(Inertia::LINEAR,Inertia::LINEAR)
        -= data.UDinv.template middleRows<3>(Inertia::LINEAR) * I_.template block<3,3> (Inertia::ANGULAR, Inertia::LINEAR);
        I_.template block<6,3>(0,Inertia::ANGULAR).setZero();
        I_.template block<3,3>(Inertia::ANGULAR,Inertia::LINEAR).setZero();
      }
    }
    
    static std::string classname() { return std::string("JointModelSpherical"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelSphericalTpl<NewScalar,Options> cast() const
    {
      typedef JointModelSphericalTpl<NewScalar,Options> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelSphericalTpl

} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelSphericalTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelSphericalTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataSphericalTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataSphericalTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_spherical_hpp__
