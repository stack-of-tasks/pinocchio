//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_revolute_hpp__
#define __pinocchio_joint_revolute_hpp__

#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"
#include "pinocchio/utils/axis-label.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis> struct MotionRevoluteTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< MotionRevoluteTpl<Scalar,Options,axis> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< MotionRevoluteTpl<Scalar,Options,axis>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< MotionRevoluteTpl<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename PINOCCHIO_EIGEN_REF_CONST_TYPE(Vector6) ToVectorConstReturnType;
    typedef typename PINOCCHIO_EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef Matrix4 HomogeneousMatrixType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    typedef MotionPlain PlainReturnType;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionRevoluteTpl
  
  template<typename Scalar, int Options, int axis> struct TransformRevoluteTpl;
  
  template<typename _Scalar, int _Options, int _axis>
  struct traits< TransformRevoluteTpl<_Scalar,_Options,_axis> >
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
    typedef Matrix3 AngularType;
    typedef Matrix3 AngularRef;
    typedef Matrix3 ConstAngularRef;
    typedef typename Vector3::ConstantReturnType LinearType;
    typedef typename Vector3::ConstantReturnType LinearRef;
    typedef const typename Vector3::ConstantReturnType ConstLinearRef;
    typedef typename traits<PlainType>::ActionMatrixType ActionMatrixType;
    typedef typename traits<PlainType>::HomogeneousMatrixType HomogeneousMatrixType;
  }; // traits TransformRevoluteTpl
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< TransformRevoluteTpl<Scalar,Options,axis> >
  { typedef typename traits <TransformRevoluteTpl<Scalar,Options,axis> >::PlainType ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct TransformRevoluteTpl : SE3Base< TransformRevoluteTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_SE3_TYPEDEF_TPL(TransformRevoluteTpl);
    
    TransformRevoluteTpl() {}
    TransformRevoluteTpl(const Scalar & sin, const Scalar & cos)
    : m_sin(sin), m_cos(cos)
    {}
    
    PlainType plain() const
    {
      PlainType res(PlainType::Identity());
      _setRotation (res.rotation());
      return res;
    }
    
    operator PlainType() const { return plain(); }
    
    template<typename S2, int O2>
    typename SE3GroupAction<TransformRevoluteTpl>::ReturnType
    se3action(const SE3Tpl<S2,O2> & m) const
    {
      typedef typename SE3GroupAction<TransformRevoluteTpl>::ReturnType ReturnType;
      ReturnType res;
      switch(axis)
      {
        case 0:
        {
          res.rotation().col(0) = m.rotation().col(0);
          res.rotation().col(1).noalias() = m_cos * m.rotation().col(1) + m_sin * m.rotation().col(2);
          res.rotation().col(2).noalias() = res.rotation().col(0).cross(res.rotation().col(1));
          break;
        }
        case 1:
        {
          res.rotation().col(2).noalias() = m_cos * m.rotation().col(2) + m_sin * m.rotation().col(0);
          res.rotation().col(1) = m.rotation().col(1);
          res.rotation().col(0).noalias() = res.rotation().col(1).cross(res.rotation().col(2));
          break;
        }
        case 2:
        {
          res.rotation().col(0).noalias() = m_cos * m.rotation().col(0) + m_sin * m.rotation().col(1);
          res.rotation().col(1).noalias() = res.rotation().col(2).cross(res.rotation().col(0));
          res.rotation().col(2) = m.rotation().col(2);
          break;
        }
        default:
        {
          assert(false && "must nerver happened");
          break;
        }
      }
      res.translation() = m.translation();
      return res;
    }
    
    const Scalar & sin() const { return m_sin; }
    Scalar & sin() { return m_sin; }
    
    const Scalar & cos() const { return m_cos; }
    Scalar & cos() { return m_cos; }
    
    template<typename OtherScalar>
    void setValues(const OtherScalar & sin, const OtherScalar & cos)
    { m_sin = sin; m_cos = cos; }

    LinearType translation() const { return LinearType::PlainObject::Zero(3); };
    AngularType rotation() const {
      AngularType m(AngularType::Identity(3));
      _setRotation (m);
      return m;
    }
    
    bool isEqual(const TransformRevoluteTpl & other) const
    {
      return m_cos == other.m_cos && m_sin == other.m_sin;
    }
    
  protected:
    
    Scalar m_sin, m_cos;
    inline void _setRotation (typename PlainType::AngularRef& rot) const
    {
      switch(axis)
      {
        case 0:
        {
          rot.coeffRef(1,1) = m_cos; rot.coeffRef(1,2) = -m_sin;
          rot.coeffRef(2,1) = m_sin; rot.coeffRef(2,2) =  m_cos;
          break;
        }
        case 1:
        {
          rot.coeffRef(0,0) =  m_cos; rot.coeffRef(0,2) = m_sin;
          rot.coeffRef(2,0) = -m_sin; rot.coeffRef(2,2) = m_cos;
          break;
        }
        case 2:
        {
          rot.coeffRef(0,0) = m_cos; rot.coeffRef(0,1) = -m_sin;
          rot.coeffRef(1,0) = m_sin; rot.coeffRef(1,1) =  m_cos;
          break;
        }
        default:
        {
          assert(false && "must nerver happened");
          break;
        }
      }
    }
  };

  template<typename _Scalar, int _Options, int axis>
  struct MotionRevoluteTpl
  : MotionBase< MotionRevoluteTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MOTION_TYPEDEF_TPL(MotionRevoluteTpl);
    typedef SpatialAxis<axis+ANGULAR> Axis;
    typedef typename Axis::CartesianAxis3 CartesianAxis3;

    MotionRevoluteTpl() {}
    
    MotionRevoluteTpl(const Scalar & w) : m_w(w)  {}
    
    template<typename Vector1Like>
    MotionRevoluteTpl(const Eigen::MatrixBase<Vector1Like> & v)
    : m_w(v[0])
    {
      using namespace Eigen;
      EIGEN_STATIC_ASSERT_SIZE_1x1(Vector1Like);
    }
    
    inline PlainReturnType plain() const { return Axis() * m_w; }
    
    template<typename OtherScalar>
    MotionRevoluteTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionRevoluteTpl(alpha*m_w);
    }
    
    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & m) const
    {
      m.linear().setZero();
      for(Eigen::DenseIndex k = 0; k < 3; ++k)
        m.angular()[k] = k == axis ? m_w : (Scalar)0;
    }
    
    template<typename MotionDerived>
    inline void addTo(MotionDense<MotionDerived> & v) const
    {
      typedef typename MotionDense<MotionDerived>::Scalar OtherScalar;
      v.angular()[axis] += (OtherScalar)m_w;
    }
    
    template<typename S2, int O2, typename D2>
    inline void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().noalias() = m.rotation().col(axis) * m_w;
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
    void se3ActionInverse_impl(const SE3Tpl<S2,O2> & m,
                               MotionDense<D2> & v) const
    {
      // Linear
      CartesianAxis3::alphaCross(m_w,m.translation(),v.angular());
      v.linear().noalias() = m.rotation().transpose() * v.angular();
      
      // Angular
      v.angular().noalias() = m.rotation().transpose().col(axis) * m_w;
    }
    
    template<typename S2, int O2>
    MotionPlain se3ActionInverse_impl(const SE3Tpl<S2,O2> & m) const
    {
      MotionPlain res;
      se3ActionInverse_impl(m,res);
      return res;
    }
    
    template<typename M1, typename M2>
    EIGEN_STRONG_INLINE
    void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
    {
      // Linear
      CartesianAxis3::alphaCross(-m_w,v.linear(),mout.linear());

      // Angular
      CartesianAxis3::alphaCross(-m_w,v.angular(),mout.angular());
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    Scalar & angularRate() { return m_w; }
    const Scalar & angularRate() const { return m_w; }
    
    bool isEqual_impl(const MotionRevoluteTpl & other) const
    {
      return m_w == other.m_w;
    }
    
  protected:
    
    Scalar m_w;
  }; // struct MotionRevoluteTpl

  template<typename S1, int O1, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionRevoluteTpl<S1,O1,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }
  
  template<typename MotionDerived, typename S2, int O2, int axis>
  EIGEN_STRONG_INLINE
  typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteTpl<S2,O2,axis>& m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options, int axis> struct ConstraintRevoluteTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< ConstraintRevoluteTpl<Scalar,Options,axis> >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< ConstraintRevoluteTpl<Scalar,Options,axis>, MotionDerived >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
  template<typename Scalar, int Options, int axis, typename ForceDerived>
  struct ConstraintForceOp< ConstraintRevoluteTpl<Scalar,Options,axis>, ForceDerived>
  { typedef typename ForceDense<ForceDerived>::ConstAngularType::template ConstFixedSegmentReturnType<1>::Type ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename ForceSet>
  struct ConstraintForceSetOp< ConstraintRevoluteTpl<Scalar,Options,axis>, ForceSet>
  { typedef typename Eigen::MatrixBase<ForceSet>::ConstRowXpr ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct traits< ConstraintRevoluteTpl<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef MotionRevoluteTpl<Scalar,Options,axis> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintRevoluteTpl

  template<typename _Scalar, int _Options, int axis>
  struct ConstraintRevoluteTpl
  : ConstraintBase< ConstraintRevoluteTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(ConstraintRevoluteTpl)
    enum { NV = 1 };
    
    typedef SpatialAxis<ANGULAR+axis> Axis;
    
    ConstraintRevoluteTpl() {}

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    { return JointMotion(v[0]); }
    
    template<typename S1, int O1>
    typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType ReturnType;
      ReturnType res;
      res.template segment<3>(LINEAR) = m.translation().cross(m.rotation().col(axis));
      res.template segment<3>(ANGULAR) = m.rotation().col(axis);
      return res;
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType ReturnType;
      typedef typename Axis::CartesianAxis3 CartesianAxis3;
      ReturnType res;
      res.template segment<3>(LINEAR).noalias() = m.rotation().transpose()*CartesianAxis3::cross(m.translation());
      res.template segment<3>(ANGULAR) = m.rotation().transpose().col(axis);
      return res;
    }

    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      const ConstraintRevoluteTpl & ref;
      TransposeConst(const ConstraintRevoluteTpl & ref) : ref(ref) {}

      template<typename ForceDerived>
      typename ConstraintForceOp<ConstraintRevoluteTpl,ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      { return f.angular().template segment<1>(axis); }

      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<ConstraintRevoluteTpl,Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        assert(F.rows()==6);
        return F.row(ANGULAR + axis);
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
    typename MotionAlgebraAction<ConstraintRevoluteTpl,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename MotionAlgebraAction<ConstraintRevoluteTpl,MotionDerived>::ReturnType ReturnType;
      ReturnType res;
      MotionRef<ReturnType> v(res);
      v = m.cross(Axis());
      return res;
    }
    
    bool isEqual(const ConstraintRevoluteTpl &) const { return true; }
    
  }; // struct ConstraintRevoluteTpl

  template<typename _Scalar, int _Options, int _axis>
  struct JointRevoluteTpl
  {
    typedef _Scalar Scalar;
    
    enum
    {
      Options = _Options,
      axis = _axis
    };
  };
  
  template<typename S1, int O1,typename S2, int O2, int axis>
  struct MultiplicationOp<InertiaTpl<S1,O1>, ConstraintRevoluteTpl<S2,O2,axis> >
  {
    typedef Eigen::Matrix<S2,6,1,O2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintRevoluteTpl<S2,O2,0> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintRevoluteTpl<S2,O2,0> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,3) = ( 0,-z, y,  I00+yy+zz,  I01-xy   ,  I02-xz   ) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        const typename Inertia::Symmetric3 & I = Y.inertia();
        
        res <<
        (S2)0,
        -m*z,
        m*y,
        I(0,0)+m*(y*y+z*z),
        I(0,1)-m*x*y,
        I(0,2)-m*x*z;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintRevoluteTpl<S2,O2,1> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintRevoluteTpl<S2,O2,1> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,4) = ( z, 0,-x,  I10-xy   ,  I11+xx+zz,  I12-yz   ) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        const typename Inertia::Symmetric3 & I = Y.inertia();
        
        res <<
        m*z,
        (S2)0,
        -m*x,
        I(1,0)-m*x*y,
        I(1,1)+m*(x*x+z*z),
        I(1,2)-m*y*z;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, ConstraintRevoluteTpl<S2,O2,2> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef ConstraintRevoluteTpl<S2,O2,2> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & /*constraint*/)
      {
        ReturnType res;
        
        /* Y(:,5) = (-y, x, 0,  I20-xz   ,  I21-yz   ,  I22+xx+yy) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        const typename Inertia::Symmetric3 & I = Y.inertia();
        
        res <<
        -m*y,
        m*x,
        (S2)0,
        I(2,0)-m*x*z,
        I(2,1)-m*y*z,
        I(2,2)+m*(x*x+y*y);
        
        return res;
      }
    };
  } // namespace impl
  
  template<typename M6Like,typename S2, int O2, int axis>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintRevoluteTpl<S2,O2,axis> >
  {
    typedef typename M6Like::ConstColXpr ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options, int axis>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, ConstraintRevoluteTpl<Scalar,Options,axis> >
    {
      typedef ConstraintRevoluteTpl<Scalar,Options,axis> Constraint;
      typedef typename MultiplicationOp<Eigen::MatrixBase<M6Like>,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y,
                                   const Constraint & /*constraint*/)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
        return Y.col(Inertia::ANGULAR + axis);
      }
    };
  } // namespace impl

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointRevoluteTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataRevoluteTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelRevoluteTpl<Scalar,Options,axis> JointModelDerived;
    typedef ConstraintRevoluteTpl<Scalar,Options,axis> Constraint_t;
    typedef TransformRevoluteTpl<Scalar,Options,axis> Transformation_t;
    typedef MotionRevoluteTpl<Scalar,Options,axis> Motion_t;
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
  struct traits< JointDataRevoluteTpl<Scalar,Options,axis> >
  { typedef JointRevoluteTpl<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelRevoluteTpl<Scalar,Options,axis> >
  { typedef JointRevoluteTpl<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataRevoluteTpl : public JointDataBase< JointDataRevoluteTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteTpl<_Scalar,_Options,axis> JointDerived;
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

    JointDataRevoluteTpl()
    : M((Scalar)0,(Scalar)1)
    , v((Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    {}

    static std::string classname()
    {
      return std::string("JointDataR") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataRevoluteTpl
  
  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType< NewScalar, JointModelRevoluteTpl<Scalar,Options,axis> >
  {
    typedef JointModelRevoluteTpl<NewScalar,Options,axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelRevoluteTpl
  : public JointModelBase< JointModelRevoluteTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointModelBase<JointModelRevoluteTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    JointModelRevoluteTpl() {}

    const std::vector<bool> hasConfigurationLimit() const
    {
      return {true};
    }

    const std::vector<bool> hasConfigurationLimitInTangent() const
    {
      return {true};
    }

    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar OtherScalar;
      
      const OtherScalar & q = qs[idx_q()];
      OtherScalar ca,sa; SINCOS(q,&sa,&ca);
      data.M.setValues(sa,ca);
    }

    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());

      data.v.angularRate() = static_cast<Scalar>(vs[idx_v()]);
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = Scalar(1)/I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    static std::string classname()
    {
      return std::string("JointModelR") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteTpl<NewScalar,Options,axis> cast() const
    {
      typedef JointModelRevoluteTpl<NewScalar,Options,axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }
    
  }; // struct JointModelRevoluteTpl

  typedef JointRevoluteTpl<double,0,0> JointRX;
  typedef JointDataRevoluteTpl<double,0,0> JointDataRX;
  typedef JointModelRevoluteTpl<double,0,0> JointModelRX;

  typedef JointRevoluteTpl<double,0,1> JointRY;
  typedef JointDataRevoluteTpl<double,0,1> JointDataRY;
  typedef JointModelRevoluteTpl<double,0,1> JointModelRY;

  typedef JointRevoluteTpl<double,0,2> JointRZ;
  typedef JointDataRevoluteTpl<double,0,2> JointDataRZ;
  typedef JointModelRevoluteTpl<double,0,2> JointModelRZ;

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointModelRevoluteTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointModelRevoluteTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointDataRevoluteTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointDataRevoluteTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_revolute_hpp__
