//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_translation_hpp__
#define __pinocchio_joint_translation_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options=0> struct MotionTranslationTpl;
  typedef MotionTranslationTpl<double> MotionTranslation;
  
  template<typename Scalar, int Options>
  struct SE3GroupAction< MotionTranslationTpl<Scalar,Options> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, typename MotionDerived>
  struct MotionAlgebraAction< MotionTranslationTpl<Scalar,Options>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options>
  struct traits< MotionTranslationTpl<_Scalar,_Options> >
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
  }; // traits MotionTranslationTpl

  template<typename _Scalar, int _Options>
  struct MotionTranslationTpl
  : MotionBase< MotionTranslationTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MOTION_TYPEDEF_TPL(MotionTranslationTpl);

    MotionTranslationTpl() {}
    
    template<typename Vector3Like>
    MotionTranslationTpl(const Eigen::MatrixBase<Vector3Like> & v)
    : m_v(v)
    {}
    
    MotionTranslationTpl(const MotionTranslationTpl & other)
    : m_v(other.m_v)
    {}
 
    Vector3 & operator()() { return m_v; }
    const Vector3 & operator()() const { return m_v; }
    
    inline PlainReturnType plain() const
    {
      return PlainReturnType(m_v,PlainReturnType::Vector3::Zero());
    }
    
    bool isEqual_impl(const MotionTranslationTpl & other) const
    {
      return m_v == other.m_v;
    }
    
    MotionTranslationTpl & operator=(const MotionTranslationTpl & other)
    {
      m_v = other.m_v;
      return *this;
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & other) const
    {
      other.linear() += m_v;
    }
    
    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear() = m_v;
      other.angular().setZero();
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().setZero();
      v.linear().noalias() = m.rotation() * m_v; // TODO: check efficiency
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
      v.linear().noalias() = m.rotation().transpose() * m_v;
      
      // Angular
      v.angular().setZero();
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
      mout.linear().noalias() = v.angular().cross(m_v);
      
      // Angular
      mout.angular().setZero();
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    const Vector3 & linear() const { return m_v; }
    Vector3 & linear() { return m_v; }
    
  protected:
    
    Vector3 m_v;
    
  }; // struct MotionTranslationTpl
  
  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionTranslationTpl<S1,O1> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    return typename MotionDerived::MotionPlain(m2.linear() + m1.linear(), m2.angular());
  }
  
  template<typename Scalar, int Options> struct TransformTranslationTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< TransformTranslationTpl<_Scalar,_Options> >
  {
    enum {
      Options = _Options,
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef _Scalar Scalar;
    typedef SE3Tpl<Scalar,Options> PlainType;
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef typename Matrix3::IdentityReturnType AngularType;
    typedef AngularType AngularRef;
    typedef AngularType ConstAngularRef;
    typedef Vector3 LinearType;
    typedef LinearType & LinearRef;
    typedef const LinearType & ConstLinearRef;
    typedef typename traits<PlainType>::ActionMatrixType ActionMatrixType;
    typedef typename traits<PlainType>::HomogeneousMatrixType HomogeneousMatrixType;
  }; // traits TransformTranslationTpl
  
  template<typename Scalar, int Options>
  struct SE3GroupAction< TransformTranslationTpl<Scalar,Options> >
  { typedef typename traits <TransformTranslationTpl<Scalar,Options> >::PlainType ReturnType; };

  template<typename _Scalar, int _Options>
  struct TransformTranslationTpl
  : SE3Base< TransformTranslationTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_SE3_TYPEDEF_TPL(TransformTranslationTpl);
    typedef typename traits<TransformTranslationTpl>::Vector3 Vector3;
    
    TransformTranslationTpl() {}
    
    template<typename Vector3Like>
    TransformTranslationTpl(const Eigen::MatrixBase<Vector3Like> & translation)
    : m_translation(translation)
    {}
    
    PlainType plain() const
    {
      PlainType res(PlainType::Identity());
      res.rotation().setIdentity();
      res.translation() = translation();
      
      return res;
    }
    
    operator PlainType() const { return plain(); }
    
    template<typename S2, int O2>
    typename SE3GroupAction<TransformTranslationTpl>::ReturnType
    se3action(const SE3Tpl<S2,O2> & m) const
    {
      typedef typename SE3GroupAction<TransformTranslationTpl>::ReturnType ReturnType;
      ReturnType res(m);
      res.translation() += translation();
      
      return res;
    }
    
    ConstLinearRef translation() const { return m_translation; }
    LinearRef translation() { return m_translation; }
    
    AngularType rotation() const { return AngularType(3,3); }
    
    bool isEqual(const TransformTranslationTpl & other) const
    {
      return m_translation == other.m_translation;
    }
    
  protected:
    
    LinearType m_translation;
  };
  
  template<typename Scalar, int Options> struct ConstraintTranslationTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintTranslationTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    
    enum { Options = _Options };
    enum { LINEAR = 0, ANGULAR = 3 };
    
    typedef MotionTranslationTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,3,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,3,Options> DenseBase;
    
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintTranslationTpl
  
  template<typename _Scalar, int _Options>
  struct ConstraintTranslationTpl
  : ConstraintBase< ConstraintTranslationTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintTranslationTpl)
    
    enum { NV = 3 };
    
    ConstraintTranslationTpl() {}
    
//    template<typename S1, int O1>
//    Motion operator*(const MotionTranslationTpl<S1,O1> & vj) const
//    { return Motion(vj(), Motion::Vector3::Zero()); }
    
    template<typename Vector3Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector3Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return JointMotion(v);
    }
    
    int nv_impl() const { return NV; }
    
    struct ConstraintTranspose
    {
      const ConstraintTranslationTpl & ref;
      ConstraintTranspose(const ConstraintTranslationTpl & ref) : ref(ref) {}
      
      template<typename Derived>
      typename ForceDense<Derived>::ConstLinearType
      operator* (const ForceDense<Derived> & phi)
      {
        return phi.linear();
      }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename MatrixDerived>
      const typename SizeDepType<3>::RowsReturn<MatrixDerived>::ConstType
      operator*(const Eigen::MatrixBase<MatrixDerived> & F) const
      {
        assert(F.rows()==6);
        return F.derived().template middleRows<3>(LINEAR);
      }
      
    }; // struct ConstraintTranspose
    
    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }
    
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template middleRows<3>(LINEAR).setIdentity();
      S.template middleRows<3>(ANGULAR).setZero();
      return S;
    }
    
    template<typename S1, int O1>
    Eigen::Matrix<S1,6,3,O1> se3Action(const SE3Tpl<S1,O1> & m) const
    {
      Eigen::Matrix<S1,6,3,O1> M;
      M.template middleRows<3>(LINEAR) = m.rotation();
      M.template middleRows<3>(ANGULAR).setZero();
      
      return M;
    }
    
    template<typename S1, int O1>
    Eigen::Matrix<S1,6,3,O1> se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      Eigen::Matrix<S1,6,3,O1> M;
      M.template middleRows<3>(LINEAR) = m.rotation().transpose();
      M.template middleRows<3>(ANGULAR).setZero();
      
      return M;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstAngularType w = m.angular();
      
      DenseBase res;
      skew(w,res.template middleRows<3>(LINEAR));
      res.template middleRows<3>(ANGULAR).setZero();
      
      return res;
    }
    
    bool isEqual(const ConstraintTranslationTpl &) const { return true; }
    
  }; // struct ConstraintTranslationTpl
  
  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionTranslationTpl<S2,O2> & m2)
  {
    return m2.motionAction(m1);
  }
  
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,3,O2>
  operator*(const InertiaTpl<S1,O1> & Y,
            const ConstraintTranslationTpl<S2,O2> &)
  {
    typedef ConstraintTranslationTpl<S2,O2> Constraint;
    Eigen::Matrix<S2,6,3,O2> M;
    alphaSkew(Y.mass(),Y.lever(),M.template middleRows<3>(Constraint::ANGULAR));
    M.template middleRows<3>(Constraint::LINEAR).setZero();
    M.template middleRows<3>(Constraint::LINEAR).diagonal().fill(Y.mass ());
    
    return M;
  }
  
  /* [ABA] Y*S operator*/
  template<typename M6Like, typename S2, int O2>
  inline const typename SizeDepType<3>::ColsReturn<M6Like>::ConstType
  operator*(const Eigen::MatrixBase<M6Like> & Y,
            const ConstraintTranslationTpl<S2,O2> &)
  {
    typedef ConstraintTranslationTpl<S2,O2> Constraint;
    return Y.derived().template middleCols<3>(Constraint::LINEAR);
  }
  
  template<typename S1, int O1>
  struct SE3GroupAction< ConstraintTranslationTpl<S1,O1> >
  { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  
  template<typename S1, int O1, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintTranslationTpl<S1,O1>,MotionDerived >
  { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };

  template<typename Scalar, int Options> struct JointTranslationTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointTranslationTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 3,
      NV = 3
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataTranslationTpl<Scalar,Options> JointDataDerived;
    typedef JointModelTranslationTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintTranslationTpl<Scalar,Options> Constraint_t;
    typedef TransformTranslationTpl<Scalar,Options> Transformation_t;
    typedef MotionTranslationTpl<Scalar,Options> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  }; // traits JointTranslationTpl
  
  template<typename Scalar, int Options>
  struct traits< JointDataTranslationTpl<Scalar,Options> >
  { typedef JointTranslationTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits< JointModelTranslationTpl<Scalar,Options> >
  { typedef JointTranslationTpl<Scalar,Options> JointDerived; };
  
  template<typename _Scalar, int _Options>
  struct JointDataTranslationTpl
  : public JointDataBase< JointDataTranslationTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTranslationTpl<_Scalar,_Options> JointDerived;
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

    JointDataTranslationTpl()
    : M(Transformation_t::Vector3::Zero())
    , v(Motion_t::Vector3::Zero())
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}

    static std::string classname() { return std::string("JointDataTranslation"); }
    std::string shortname() const { return classname(); }
  }; // struct JointDataTranslationTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelTranslationTpl);
  template<typename _Scalar, int _Options>
  struct JointModelTranslationTpl
  : public JointModelBase< JointModelTranslationTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTranslationTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelTranslationTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.M.translation() = this->jointConfigSelector(qs);
    }
    
    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      data.v.linear() = this->jointVelocitySelector(vs);
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U = I.template middleCols<3>(Inertia::LINEAR);
      
      // compute inverse
//      data.Dinv.setIdentity();
//      data.U.template middleRows<3>(Inertia::LINEAR).llt().solveInPlace(data.Dinv);
      internal::PerformStYSInversion<Scalar>::run(data.U.template middleRows<3>(Inertia::LINEAR),data.Dinv);
      
      data.UDinv.template middleRows<3>(Inertia::LINEAR).setIdentity(); // can be put in data constructor
      data.UDinv.template middleRows<3>(Inertia::ANGULAR).noalias() = data.U.template middleRows<3>(Inertia::ANGULAR) * data.Dinv;
      
      if (update_I)
      {
        Matrix6Like & I_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I);
        I_.template block<3,3>(Inertia::ANGULAR,Inertia::ANGULAR)
        -= data.UDinv.template middleRows<3>(Inertia::ANGULAR) * I_.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        I_.template middleCols<3>(Inertia::LINEAR).setZero();
        I_.template block<3,3>(Inertia::LINEAR,Inertia::ANGULAR).setZero();
      }
    }
    
    static std::string classname() { return std::string("JointModelTranslation"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelTranslationTpl<NewScalar,Options> cast() const
    {
      typedef JointModelTranslationTpl<NewScalar,Options> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelTranslationTpl
  
} // namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelTranslationTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelTranslationTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataTranslationTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataTranslationTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_translation_hpp__
