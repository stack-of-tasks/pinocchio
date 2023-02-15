//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_multibody_joint_Universal_hpp__
#define __pinocchio_multibody_joint_Universal_hpp__

#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"
#include "pinocchio/utils/axis-label.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis> struct MotionUniversalTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< MotionUniversalTpl<Scalar,Options,axis> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< MotionUniversalTpl<Scalar,Options,axis>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< MotionUniversalTpl<_Scalar,_Options,axis> >
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
  }; // traits MotionUniversalTpl
  
  template<typename Scalar, int Options, int axis> struct TransformUniversalTpl;
  
  template<typename _Scalar, int _Options, int _axis>
  struct traits< TransformUniversalTpl<_Scalar,_Options,_axis> >
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
  }; // traits TransformUniversalTpl
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< TransformUniversalTpl<Scalar,Options,axis> >
  { typedef typename traits <TransformUniversalTpl<Scalar,Options,axis> >::PlainType ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct TransformUniversalTpl : SE3Base< TransformUniversalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_SE3_TYPEDEF_TPL(TransformUniversalTpl);
    
    TransformUniversalTpl() {}
    TransformUniversalTpl(const Scalar & sin, const Scalar & cos)
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
    typename SE3GroupAction<TransformUniversalTpl>::ReturnType
    se3action(const SE3Tpl<S2,O2> & m) const
    {
      typedef typename SE3GroupAction<TransformUniversalTpl>::ReturnType ReturnType;
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
          assert(false && "must never happened");
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

    LinearType translation() const
    {
      return LinearType::PlainObject::Zero(3);
    }
    AngularType rotation() const
    {
      AngularType m(AngularType::Identity(3));
      _setRotation (m);
      return m;
    }
    
    bool isEqual(const TransformUniversalTpl & other) const
    {
      return internal::comparison_eq(m_cos, other.m_cos) &&
	internal::comparison_eq(m_sin, other.m_sin);
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
          assert(false && "must never happened");
          break;
        }
      }
    }
  };

  template<typename _Scalar, int _Options, int axis>
  struct MotionUniversalTpl
  : MotionBase< MotionUniversalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MOTION_TYPEDEF_TPL(MotionUniversalTpl);
    typedef SpatialAxis<axis+ANGULAR> Axis;
    typedef typename Axis::CartesianAxis3 CartesianAxis3;

    MotionUniversalTpl() {}
    
    MotionUniversalTpl(const Scalar & w) : m_w(w)  {}
    
    template<typename Vector1Like>
    MotionUniversalTpl(const Eigen::MatrixBase<Vector1Like> & v)
    : m_w(v[0])
    {
      using namespace Eigen;
      EIGEN_STATIC_ASSERT_SIZE_1x1(Vector1Like);
    }
    
    inline PlainReturnType plain() const { return Axis() * m_w; }
    
    template<typename OtherScalar>
    MotionUniversalTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionUniversalTpl(alpha*m_w);
    }
    
    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & m) const
    {
      m.linear().setZero();
      for(Eigen::DenseIndex k = 0; k < 3; ++k){
        m.angular()[k] = k == axis ? m_w : Scalar(0);
      }
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
    
    bool isEqual_impl(const MotionUniversalTpl & other) const
    {
      return internal::comparison_eq(m_w, other.m_w);
    }
    
  protected:
    
    Scalar m_w;
  }; // struct MotionUniversalTpl

  template<typename S1, int O1, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionUniversalTpl<S1,O1,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }
  
  template<typename MotionDerived, typename S2, int O2, int axis>
  EIGEN_STRONG_INLINE
  typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionUniversalTpl<S2,O2,axis>& m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options, int axis> struct JointMotionSubspaceUniversalTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< JointMotionSubspaceUniversalTpl<Scalar,Options,axis> >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< JointMotionSubspaceUniversalTpl<Scalar,Options,axis>, MotionDerived >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
  template<typename Scalar, int Options, int axis, typename ForceDerived>
  struct ConstraintForceOp< JointMotionSubspaceUniversalTpl<Scalar,Options,axis>, ForceDerived>
  { typedef typename ForceDense<ForceDerived>::ConstAngularType::template ConstFixedSegmentReturnType<1>::Type ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename ForceSet>
  struct ConstraintForceSetOp< JointMotionSubspaceUniversalTpl<Scalar,Options,axis>, ForceSet>
  { typedef typename Eigen::MatrixBase<ForceSet>::ConstRowXpr ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointMotionSubspaceUniversalTpl<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    
    typedef MotionUniversalTpl<Scalar,Options,axis> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef Eigen::Matrix<Scalar,1,1,Options> ReducedSquaredMatrix;
    
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
    
    typedef typename ReducedSquaredMatrix::IdentityReturnType StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceUniversalTpl

  template<typename _Scalar, int _Options, int axis>
  struct JointMotionSubspaceUniversalTpl
  : JointMotionSubspaceBase< JointMotionSubspaceUniversalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceUniversalTpl)
    enum { NV = 1 };
    
    typedef SpatialAxis<ANGULAR+axis> Axis;
    
    JointMotionSubspaceUniversalTpl() {}

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    { return JointMotion(v[0]); }
    
    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType ReturnType;
      ReturnType res;
      res.template segment<3>(LINEAR) = m.translation().cross(m.rotation().col(axis));
      res.template segment<3>(ANGULAR) = m.rotation().col(axis);
      return res;
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceUniversalTpl>::ReturnType ReturnType;
      typedef typename Axis::CartesianAxis3 CartesianAxis3;
      ReturnType res;
      res.template segment<3>(LINEAR).noalias() = m.rotation().transpose()*CartesianAxis3::cross(m.translation());
      res.template segment<3>(ANGULAR) = m.rotation().transpose().col(axis);
      return res;
    }

    int nv_impl() const { return NV; }
    
    struct TransposeConst : JointMotionSubspaceTransposeBase<JointMotionSubspaceUniversalTpl>
    {
      const JointMotionSubspaceUniversalTpl & ref;
      TransposeConst(const JointMotionSubspaceUniversalTpl & ref) : ref(ref) {}

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceUniversalTpl,ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      { return f.angular().template segment<1>(axis); }

      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<JointMotionSubspaceUniversalTpl,Derived>::ReturnType
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
    typename MotionAlgebraAction<JointMotionSubspaceUniversalTpl,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename MotionAlgebraAction<JointMotionSubspaceUniversalTpl,MotionDerived>::ReturnType ReturnType;
      ReturnType res;
      MotionRef<ReturnType> v(res);
      v = m.cross(Axis());
      return res;
    }
    
    bool isEqual(const JointMotionSubspaceUniversalTpl &) const { return true; }
    
  }; // struct JointMotionSubspaceUniversalTpl

  template<typename _Scalar, int _Options, int _axis>
  struct JointUniversalTpl
  {
    typedef _Scalar Scalar;
    
    enum
    {
      Options = _Options,
      axis = _axis
    };
  };
  
  template<typename S1, int O1,typename S2, int O2, int axis>
  struct MultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceUniversalTpl<S2,O2,axis> >
  {
    typedef Eigen::Matrix<S2,6,1,O2> ReturnType;
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceUniversalTpl<S2,O2,0> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceUniversalTpl<S2,O2,0> Constraint;
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
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceUniversalTpl<S2,O2,1> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceUniversalTpl<S2,O2,1> Constraint;
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
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceUniversalTpl<S2,O2,2> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceUniversalTpl<S2,O2,2> Constraint;
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
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, JointMotionSubspaceUniversalTpl<S2,O2,axis> >
  {
    typedef typename M6Like::ConstColXpr ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options, int axis>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, JointMotionSubspaceUniversalTpl<Scalar,Options,axis> >
    {
      typedef JointMotionSubspaceUniversalTpl<Scalar,Options,axis> Constraint;
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
  struct traits< JointUniversalTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataUniversalTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelUniversalTpl<Scalar,Options,axis> JointModelDerived;
    typedef JointMotionSubspaceUniversalTpl<Scalar,Options,axis> Constraint_t;
    typedef TransformUniversalTpl<Scalar,Options,axis> Transformation_t;
    typedef MotionUniversalTpl<Scalar,Options,axis> Motion_t;
    typedef MotionZeroTpl<Scalar,Options> Bias_t;

    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointDataUniversalTpl<_Scalar,_Options,axis> >
  {
    typedef JointUniversalTpl<_Scalar,_Options,axis> JointDerived;
    typedef _Scalar Scalar;
  };
  
  template<typename _Scalar, int _Options, int axis>
  struct traits< JointModelUniversalTpl<_Scalar,_Options,axis> >
  {
    typedef JointUniversalTpl<_Scalar,_Options,axis> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataUniversalTpl
  : public JointDataBase< JointDataUniversalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointUniversalTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE(JointDerived);
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR
    
    ConfigVector_t joint_q;
    TangentVector_t joint_v;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;
    D_t StU;

    JointDataUniversalTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , M((Scalar)0,(Scalar)1)
    , v((Scalar)0)
    , U(U_t::Zero())
    , Dinv(D_t::Zero())
    , UDinv(UD_t::Zero())
    , StU(D_t::Zero())
    {}

    static std::string classname()
    {
      return std::string("JointDataR") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataUniversalTpl
  
  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType< NewScalar, JointModelUniversalTpl<Scalar,Options,axis> >
  {
    typedef JointModelUniversalTpl<NewScalar,Options,axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelUniversalTpl
  : public JointModelBase< JointModelUniversalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointUniversalTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointModelBase<JointModelUniversalTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    JointModelUniversalTpl() {}
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q[0] = qs[idx_q()];
      Scalar ca,sa; SINCOS(data.joint_q[0],&sa,&ca);
      data.M.setValues(sa,ca);
    }

    template<typename ConfigVector, typename TangentVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());

      data.joint_v[0] = vs[idx_v()];
      data.v.angularRate() = data.joint_v[0];
    }
    
    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<VectorLike> & armature,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = Scalar(1)/(I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis) + armature[0]);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I).noalias() -= data.UDinv * data.U.transpose();
    }
    
    static std::string classname()
    {
      return std::string("JointModelR") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelUniversalTpl<NewScalar,Options,axis> cast() const
    {
      typedef JointModelUniversalTpl<NewScalar,Options,axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }
    
  }; // struct JointModelUniversalTpl

  typedef JointUniversalTpl<context::Scalar,context::Options,0> JointUX;
  typedef JointDataUniversalTpl<context::Scalar,context::Options,0> JointDataUX;
  typedef JointModelUniversalTpl<context::Scalar,context::Options,0> JointModelUX;

  typedef JointUniversalTpl<context::Scalar,context::Options,1> JointUY;
  typedef JointDataUniversalTpl<context::Scalar,context::Options,1> JointDataUY;
  typedef JointModelUniversalTpl<context::Scalar,context::Options,1> JointModelUY;

  typedef JointUniversalTpl<context::Scalar,context::Options,2> JointUZ;
  typedef JointDataUniversalTpl<context::Scalar,context::Options,2> JointDataUZ;
  typedef JointModelUniversalTpl<context::Scalar,context::Options,2> JointModelUZ;

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointModelUniversalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointModelUniversalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointDataUniversalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointDataUniversalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_multibody_joint_Universal_hpp__
