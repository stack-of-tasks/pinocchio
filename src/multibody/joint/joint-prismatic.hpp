//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_prismatic_hpp__
#define __pinocchio_joint_prismatic_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"
#include "pinocchio/utils/axis-label.hpp"

namespace pinocchio
{
  
  template<typename Scalar, int Options, int _axis> struct MotionPrismaticTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< MotionPrismaticTpl<Scalar,Options,axis> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< MotionPrismaticTpl<Scalar,Options,axis>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options, int _axis>
  struct traits < MotionPrismaticTpl<_Scalar,_Options,_axis> >
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
  }; // struct traits MotionPrismaticTpl

  template<typename _Scalar, int _Options, int _axis>
  struct MotionPrismaticTpl
  : MotionBase < MotionPrismaticTpl<_Scalar,_Options,_axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionPrismaticTpl);
    
    enum { axis = _axis };
    
    typedef SpatialAxis<_axis+LINEAR> Axis;
    typedef typename Axis::CartesianAxis3 CartesianAxis3;

    MotionPrismaticTpl() {}
    MotionPrismaticTpl(const Scalar & v) : m_v(v) {}

    inline PlainReturnType plain() const { return Axis() * m_v; }
    
    template<typename OtherScalar>
    MotionPrismaticTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionPrismaticTpl(alpha*m_v);
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & other) const
    {
      typedef typename MotionDense<Derived>::Scalar OtherScalar;
      other.linear()[_axis] += (OtherScalar) m_v;
    }
    
    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & other) const
    {
      for(Eigen::DenseIndex k = 0; k < 3; ++k)
        other.linear()[k] = k == axis ? m_v : (Scalar)0;
      other.angular().setZero();
    }
    
    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().setZero();
      v.linear().noalias() = m_v * (m.rotation().col(axis));
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
      v.linear().noalias() = m_v * (m.rotation().transpose().col(axis));
      
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
      CartesianAxis3::alphaCross(-m_v,v.angular(),mout.linear());

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
    
    Scalar & linearRate() { return m_v; }
    const Scalar & linearRate() const { return m_v; }
    
    bool isEqual_impl(const MotionPrismaticTpl & other) const
    {
      return m_v == other.m_v;
    }
    
  protected:
    
    Scalar m_v;
  }; // struct MotionPrismaticTpl

  template<typename Scalar, int Options, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionPrismaticTpl<Scalar,Options,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }
  
  template<typename MotionDerived, typename S2, int O2, int axis>
  EIGEN_STRONG_INLINE
  typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionPrismaticTpl<S2,O2,axis> & m2)
  {
    return m2.motionAction(m1);
  }
  
  template<typename Scalar, int Options, int axis> struct TransformPrismaticTpl;
  
  template<typename _Scalar, int _Options, int _axis>
  struct traits< TransformPrismaticTpl<_Scalar,_Options,_axis> >
  {
    enum {
      axis = _axis,
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
    typedef const Vector3 LinearRef;
    typedef const Vector3 ConstLinearRef;
    typedef typename traits<PlainType>::ActionMatrixType ActionMatrixType;
    typedef typename traits<PlainType>::HomogeneousMatrixType HomogeneousMatrixType;
  }; // traits TransformPrismaticTpl
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< TransformPrismaticTpl<Scalar,Options,axis> >
  { typedef typename traits <TransformPrismaticTpl<Scalar,Options,axis> >::PlainType ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct TransformPrismaticTpl : SE3Base< TransformPrismaticTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_SE3_TYPEDEF_TPL(TransformPrismaticTpl);
    
    typedef SpatialAxis<axis+LINEAR> Axis;
    typedef typename Axis::CartesianAxis3 CartesianAxis3;
    
    TransformPrismaticTpl() {}
    TransformPrismaticTpl(const Scalar & displacement)
    : m_displacement(displacement)
    {}
    
    PlainType plain() const
    {
      PlainType res(PlainType::Identity());
      res.rotation().setIdentity();
      res.translation()[axis] = m_displacement;
      
      return res;
    }
    
    operator PlainType() const { return plain(); }
    
    template<typename S2, int O2>
    typename SE3GroupAction<TransformPrismaticTpl>::ReturnType
    se3action(const SE3Tpl<S2,O2> & m) const
    {
      typedef typename SE3GroupAction<TransformPrismaticTpl>::ReturnType ReturnType;
      ReturnType res(m);
      res.translation()[axis] += m_displacement;
      
      return res;
    }
    
    const Scalar & displacement() const { return m_displacement; }
    Scalar & displacement() { return m_displacement; }
    
    ConstLinearRef translation() const { return CartesianAxis3()*displacement(); };
    AngularType rotation() const { return AngularType(3,3); }
    
    bool isEqual(const TransformPrismaticTpl & other) const
    {
      return m_displacement == other.m_displacement;
    }

  protected:
    
    Scalar m_displacement;
  };

  template<typename Scalar, int Options, int axis> struct ConstraintPrismaticTpl;
  
  template<typename _Scalar, int _Options, int axis>
  struct traits< ConstraintPrismaticTpl<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionPrismaticTpl<Scalar,Options,axis> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintRevolute
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< ConstraintPrismaticTpl<Scalar,Options,axis> >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintPrismaticTpl<Scalar,Options,axis>, MotionDerived >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };

  template<typename Scalar, int Options, int axis, typename ForceDerived>
  struct ConstraintForceOp< ConstraintPrismaticTpl<Scalar,Options,axis>, ForceDerived>
  { typedef typename ForceDense<ForceDerived>::ConstLinearType::template ConstFixedSegmentReturnType<1>::Type ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename ForceSet>
  struct ConstraintForceSetOp< ConstraintPrismaticTpl<Scalar,Options,axis>, ForceSet>
  { typedef typename Eigen::MatrixBase<ForceSet>::ConstRowXpr ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct ConstraintPrismaticTpl
  : ConstraintBase < ConstraintPrismaticTpl <_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintPrismaticTpl)
    enum { NV = 1 };
    
    typedef SpatialAxis<LINEAR+axis> Axis;
    
    ConstraintPrismaticTpl() {};

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like,1);
      assert(v.size() == 1);
      return JointMotion(v[0]);
    }

    template<typename S2, int O2>
    typename SE3GroupAction<ConstraintPrismaticTpl>::ReturnType
    se3Action(const SE3Tpl<S2,O2> & m) const
    { 
      typename SE3GroupAction<ConstraintPrismaticTpl>::ReturnType res;
      MotionRef<DenseBase> v(res);
      v.linear() = m.rotation().col(axis);
      v.angular().setZero();
      return res;
    }
    
    template<typename S2, int O2>
    typename SE3GroupAction<ConstraintPrismaticTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S2,O2> & m) const
    {
      typename SE3GroupAction<ConstraintPrismaticTpl>::ReturnType res;
      MotionRef<DenseBase> v(res);
      v.linear() = m.rotation().transpose().col(axis);
      v.angular().setZero();
      return res;
    }

    int nv_impl() const { return NV; }

    struct TransposeConst
    {
      const ConstraintPrismaticTpl & ref; 
      TransposeConst(const ConstraintPrismaticTpl & ref) : ref(ref) {}

      template<typename ForceDerived>
      typename ConstraintForceOp<ConstraintPrismaticTpl,ForceDerived>::ReturnType
      operator* (const ForceDense<ForceDerived> & f) const
      { return f.linear().template segment<1>(axis); }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename Derived>
      typename ConstraintForceSetOp<ConstraintPrismaticTpl,Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F )
      {
        assert(F.rows()==6);
        return F.row(LINEAR+axis);
      }

    }; // struct TransposeConst
    TransposeConst transpose() const { return TransposeConst(*this); }

    /* CRBA joint operators
     *   - ForceSet::Block = ForceSet
     *   - ForceSet operator* (Inertia Y,Constraint S)
     *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
     *   - SE3::act(ForceSet::Block)
     */
    DenseBase matrix_impl() const
    {
      DenseBase S;
      MotionRef<DenseBase> v(S);
      v << Axis();
      return S;
    }
    
    template<typename MotionDerived>
    typename MotionAlgebraAction<ConstraintPrismaticTpl,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typename MotionAlgebraAction<ConstraintPrismaticTpl,MotionDerived>::ReturnType res;
      MotionRef<DenseBase> v(res);
      v = m.cross(Axis());
      return res;
    }
    
    bool isEqual(const ConstraintPrismaticTpl &) const { return true; }

  }; // struct ConstraintPrismaticTpl
  
  template<typename S1, int O1,typename S2, int O2, int axis>
  struct MultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticTpl<S2,O2,axis> >
  {
    typedef Eigen::Matrix<S2,6,1,O2> ReturnType;
  };
  
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticTpl<S2,O2,0> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintPrismaticTpl<S2,O2,0> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,0) = ( 1,0, 0, 0 , z , -y ) */
        const S1
        &m = Y.mass(),
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        res << m, S1(0), S1(0), S1(0), m*z, -m*y;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticTpl<S2,O2,1> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintPrismaticTpl<S2,O2,1> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,1) = ( 0,1, 0, -z , 0 , x) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &z = Y.lever()[2];
        
        res << S1(0), m, S1(0), -m*z, S1(0), m*x;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintPrismaticTpl<S2,O2,2> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintPrismaticTpl<S2,O2,2> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,2) = ( 0,0, 1, y , -x , 0) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1];
        
        res << S1(0), S1(0), m, m*y, -m*x, S1(0);
        
        return res;
      }
    };
  } // namespace impl
  
  template<typename M6Like,typename S2, int O2, int axis>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintPrismaticTpl<S2,O2,axis> >
  {
    typedef typename M6Like::ConstColXpr ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options, int axis>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintPrismaticTpl<Scalar,Options,axis> >
    {
      typedef ConstraintPrismaticTpl<Scalar,Options,axis> Constraint;
      typedef typename MultiplicationOp<Eigen::MatrixBase<M6Like>,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y,
                             const Constraint & /*constraint*/)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
        return Y.derived().col(Inertia::LINEAR + axis);
      }
    };
  } // namespace impl
  
  template<typename _Scalar, int _Options, int _axis>
  struct JointPrismaticTpl
  {
    typedef _Scalar Scalar;
    
    enum
    {
      Options = _Options,
      axis = _axis
    };
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointPrismaticTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataPrismaticTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelPrismaticTpl<Scalar,Options,axis> JointModelDerived;
    typedef ConstraintPrismaticTpl<Scalar,Options,axis> Constraint_t;
    typedef TransformPrismaticTpl<Scalar,Options,axis> Transformation_t;
    typedef MotionPrismaticTpl<Scalar,Options,axis> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };

  template<typename Scalar, int Options, int axis>
  struct traits< JointDataPrismaticTpl<Scalar,Options,axis> >
  { typedef JointPrismaticTpl<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelPrismaticTpl<Scalar,Options,axis> >
  { typedef JointPrismaticTpl<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataPrismaticTpl : public JointDataBase< JointDataPrismaticTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPrismaticTpl<_Scalar,_Options,axis> JointDerived;
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

    JointDataPrismaticTpl()
    : M((Scalar)0)
    , v((Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}

    static std::string classname()
    {
      return std::string("JointDataP") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }

  }; // struct JointDataPrismaticTpl
  
  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType< NewScalar, JointModelPrismaticTpl<Scalar,Options,axis> >
  {
    typedef JointModelPrismaticTpl<NewScalar,Options,axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelPrismaticTpl
  : public JointModelBase< JointModelPrismaticTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointPrismaticTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    typedef JointModelBase<JointModelPrismaticTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar Scalar;
      const Scalar & q = qs[idx_q()];
      data.M.displacement() = q;
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      typedef typename TangentVector::Scalar S2;
      const S2 & v = vs[idx_v()];
      data.v.linearRate() = v;
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    {
      data.U = I.col(Inertia::LINEAR + axis);
      data.Dinv[0] = Scalar(1)/I(Inertia::LINEAR + axis, Inertia::LINEAR + axis);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    static std::string classname()
    {
      return std::string("JointModelP") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelPrismaticTpl<NewScalar,Options,axis> cast() const
    {
      typedef JointModelPrismaticTpl<NewScalar,Options,axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelPrismaticTpl

  typedef JointPrismaticTpl<double,0,0> JointPX;
  typedef JointDataPrismaticTpl<double,0,0> JointDataPX;
  typedef JointModelPrismaticTpl<double,0,0> JointModelPX;

  typedef JointPrismaticTpl<double,0,1> JointPY;
  typedef JointDataPrismaticTpl<double,0,1> JointDataPY;
  typedef JointModelPrismaticTpl<double,0,1> JointModelPY;

  typedef JointPrismaticTpl<double,0,2> JointPZ;
  typedef JointDataPrismaticTpl<double,0,2> JointDataPZ;
  typedef JointModelPrismaticTpl<double,0,2> JointModelPZ;

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointModelPrismaticTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointModelPrismaticTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointDataPrismaticTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointDataPrismaticTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_prismatic_hpp__
