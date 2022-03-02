//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_joint_helical_hpp__
#define __pinocchio_multibody_joint_helical_hpp__

#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint-motion-subspace.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"
#include "pinocchio/utils/axis-label.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis> struct MotionHelicalTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< MotionHelicalTpl<Scalar,Options,axis> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< MotionHelicalTpl<Scalar,Options,axis>, MotionDerived>
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< MotionHelicalTpl<_Scalar,_Options,axis> >
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
  }; // traits MotionHelicalTpl

  template<typename Scalar, int Options, int axis> struct TransformHelicalTpl;
  
  template<typename _Scalar, int _Options, int _axis>
  struct traits< TransformHelicalTpl<_Scalar,_Options,_axis> >
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
  }; // traits TransformHelicalTpl
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< TransformHelicalTpl<Scalar,Options,axis> >
  { typedef typename traits <TransformHelicalTpl<Scalar,Options,axis> >::PlainType ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct TransformHelicalTpl : SE3Base< TransformHelicalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PINOCCHIO_SE3_TYPEDEF_TPL(TransformHelicalTpl);

    typedef SpatialAxis<axis+LINEAR> AxisLinear;
    typedef typename AxisLinear::CartesianAxis3 CartesianAxis3Linear;
    
    TransformHelicalTpl() {}
    TransformHelicalTpl(const Scalar & sin, const Scalar & cos, const Scalar & theta, const Scalar & h)
    : m_sin(sin), m_cos(cos), m_theta(theta), m_h(h)
    {}
  
    PlainType plain() const
    {
      PlainType res(PlainType::Identity());
      _setRotation (res.rotation());
      res.translation()[axis] = m_h * m_theta;
      return res;
    }
    
    operator PlainType() const { return plain(); }
    
    template<typename S2, int O2>
    typename SE3GroupAction<TransformHelicalTpl>::ReturnType
    se3action(const SE3Tpl<S2,O2> & m) const
    {
      typedef typename SE3GroupAction<TransformHelicalTpl>::ReturnType ReturnType;
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
          assert(false && "must never happen");
          break;
        }
      }
      res.translation() = m.translation();
      res.translation()[axis] += m_h * m_theta;
      return res;
    }
    
    const Scalar & sin() const { return m_sin; }
    Scalar & sin() { return m_sin; }
    
    const Scalar & cos() const { return m_cos; }
    Scalar & cos() { return m_cos; }

    const Scalar & h() const { return m_h; }
    Scalar & h() { return m_h; }

    const Scalar & theta() const { return m_theta; }
    Scalar & theta() { return m_theta; }
    
    template<typename OtherScalar>
    void setValues(const OtherScalar & sin, const OtherScalar & cos, const OtherScalar & theta, const OtherScalar & h)
    { m_sin = sin; m_cos = cos; m_theta = theta; m_h = h;}

    LinearType translation() const
    {
      return CartesianAxis3Linear() * m_h() * m_theta();
    }
    AngularType rotation() const
    {
      AngularType m(AngularType::Identity(3));
      _setRotation (m);
      return m;
    }
    
    bool isEqual(const TransformHelicalTpl & other) const
    {
      return m_cos == other.m_cos && m_sin == other.m_sin && m_h == other.m_h;
    }
    
  protected:
    
    Scalar m_sin, m_cos, m_theta, m_h;
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
          assert(false && "must never happen");
          break;
        }
      }
    }
  };  // struct TransformHelicalTpl

  template<typename _Scalar, int _Options, int axis>
  struct MotionHelicalTpl
  : MotionBase< MotionHelicalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MOTION_TYPEDEF_TPL(MotionHelicalTpl);
    typedef SpatialAxis<axis+ANGULAR> AxisAngular;
    typedef typename AxisAngular::CartesianAxis3 CartesianAxis3Angular;
    typedef SpatialAxis<axis+LINEAR> AxisLinear;
    typedef typename AxisLinear::CartesianAxis3 CartesianAxis3Linear;

    MotionHelicalTpl() {}
    
    MotionHelicalTpl(const Scalar & w, const Scalar & h) : m_w(w), m_h(h)  {}
    
    inline PlainReturnType plain() const
    {
      return PlainReturnType(CartesianAxis3Linear() * m_w * m_h,
                             CartesianAxis3Angular() * m_w);
    }
    
    template<typename OtherScalar>
    MotionHelicalTpl __mult__(const OtherScalar & alpha) const
    {
      return MotionHelicalTpl(alpha*m_w, m_h);
    }
    
    template<typename MotionDerived>
    void setTo(MotionDense<MotionDerived> & m) const
    {
      for(Eigen::DenseIndex k = 0; k < 3; ++k)
      {
        m.angular()[k] = k == axis ? m_w : (Scalar)0;
        m.linear()[k] = k == axis ? m_w * m_h : (Scalar)0;
      }
    }
    
    template<typename MotionDerived>
    inline void addTo(MotionDense<MotionDerived> & v) const
    {
      typedef typename MotionDense<MotionDerived>::Scalar OtherScalar;
      v.angular()[axis] += (OtherScalar)m_w;
      v.linear()[axis] += (OtherScalar)m_w * m_h;
    }
    
    template<typename S2, int O2, typename D2>
    inline void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      v.angular().noalias() = m.rotation().col(axis) * m_w;
      v.linear().noalias() = m.translation().cross(v.angular()) + m_w * m_h * (m.rotation().col(axis));
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
      CartesianAxis3Linear::alphaCross(m_w,m.translation(),v.angular());
      v.linear().noalias() = m.rotation().transpose() * v.angular() + m_w * m_h * (m.rotation().transpose().col(axis));
      
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
      CartesianAxis3Linear::alphaCross(-m_w,v.linear(),mout.linear());
      CartesianAxis3Linear::alphaCross(-m_h*m_w,v.angular(),mout.angular());
      mout.linear() += mout.angular();
      // Angular
      CartesianAxis3Angular::alphaCross(-m_w,v.angular(),mout.angular());
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

    Scalar & pitch() { return m_h; }
    const Scalar & pitch() const { return m_h; }
    
    bool isEqual_impl(const MotionHelicalTpl & other) const
    {
      return m_w == other.m_w && m_h == other.m_h;
    }
    
  protected:
    
    Scalar m_w, m_h;
  }; // struct MotionHelicalTpl
  template<typename S1, int O1, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionHelicalTpl<S1,O1,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }
  
  template<typename MotionDerived, typename S2, int O2, int axis>
  EIGEN_STRONG_INLINE
  typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionHelicalTpl<S2,O2,axis>& m2)
  {
    return m2.motionAction(m1);
  }

  template<typename Scalar, int Options, int axis> struct JointMotionSubspaceHelicalTpl;
  
  template<typename Scalar, int Options, int axis>
  struct SE3GroupAction< JointMotionSubspaceHelicalTpl<Scalar,Options,axis> >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename MotionDerived>
  struct MotionAlgebraAction< JointMotionSubspaceHelicalTpl<Scalar,Options,axis>, MotionDerived >
  { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
  template<typename Scalar, int Options, int axis, typename ForceDerived>
  struct ConstraintForceOp< JointMotionSubspaceHelicalTpl<Scalar,Options,axis>, ForceDerived>
  { typedef typename Eigen::Matrix<Scalar,1,1> ReturnType; };
  
  template<typename Scalar, int Options, int axis, typename ForceSet>
  struct ConstraintForceSetOp< JointMotionSubspaceHelicalTpl<Scalar,Options,axis>, ForceSet>
  { typedef typename Eigen::Matrix<Scalar,1,1> ReturnType; };

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointMotionSubspaceHelicalTpl<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    
    typedef MotionHelicalTpl<Scalar,Options,axis> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef Eigen::Matrix<Scalar,1,1,Options> ReducedSquaredMatrix;
    
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
    
    typedef typename ReducedSquaredMatrix::IdentityReturnType StDiagonalMatrixSOperationReturnType;
  }; // traits JointMotionSubspaceHelicalTpl

  template<typename _Scalar, int _Options, int axis>
  struct JointMotionSubspaceHelicalTpl
  : JointMotionSubspaceBase< JointMotionSubspaceHelicalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PINOCCHIO_CONSTRAINT_TYPEDEF_TPL(JointMotionSubspaceHelicalTpl)
    enum { NV = 1 };
    
    typedef SpatialAxis<ANGULAR+axis> AxisAngular;\
    typedef SpatialAxis<ANGULAR+axis> AxisLinear;\
    
    JointMotionSubspaceHelicalTpl() {}
    
    JointMotionSubspaceHelicalTpl(const Scalar & h) : m_h(h) {}

    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    { 
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like,1);
      assert(v.size() == 1);
      return JointMotion(v[0],m_h);
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceHelicalTpl>::ReturnType
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceHelicalTpl>::ReturnType ReturnType;
      ReturnType res;
      res.template segment<3>(LINEAR) = m.translation().cross(m.rotation().col(axis)) + m_h * (m.rotation().col(axis));
      res.template segment<3>(ANGULAR) = m.rotation().col(axis);
      return res;
    }
    
    template<typename S1, int O1>
    typename SE3GroupAction<JointMotionSubspaceHelicalTpl>::ReturnType
    se3ActionInverse(const SE3Tpl<S1,O1> & m) const
    {
      typedef typename SE3GroupAction<JointMotionSubspaceHelicalTpl>::ReturnType ReturnType;
      typedef typename AxisAngular::CartesianAxis3 CartesianAxis3;
      ReturnType res;
      res.template segment<3>(LINEAR).noalias() = m.rotation().transpose()*CartesianAxis3::cross(m.translation())
                                                + m.rotation().transpose().col(axis)*m_h;
      res.template segment<3>(ANGULAR) = m.rotation().transpose().col(axis);
      return res;
    }

    int nv_impl() const { return NV; }
    
    struct TransposeConst : JointMotionSubspaceTransposeBase<JointMotionSubspaceHelicalTpl>
    {
      const JointMotionSubspaceHelicalTpl & ref;
      TransposeConst(const JointMotionSubspaceHelicalTpl & ref) : ref(ref) {}

      template<typename ForceDerived>
      typename ConstraintForceOp<JointMotionSubspaceHelicalTpl,ForceDerived>::ReturnType
      operator*(const ForceDense<ForceDerived> & f) const
      { return Eigen::Matrix<Scalar,1,1>(f.angular()(axis) + f.linear()(axis) * ref.m_h); }

      /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename Derived>
      typename ConstraintForceSetOp<JointMotionSubspaceHelicalTpl,Derived>::ReturnType
      operator*(const Eigen::MatrixBase<Derived> & F) const
      {
        assert(F.rows()==6);
        return Eigen::Matrix<Scalar,1,1>((F.row(ANGULAR + axis) + F.row(LINEAR + axis) * ref.m_h));
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
      v << AxisAngular();
      S(LINEAR + axis) = m_h;
      return S;
    }
    
    template<typename MotionDerived>
    typename MotionAlgebraAction<JointMotionSubspaceHelicalTpl,MotionDerived>::ReturnType
    motionAction(const MotionDense<MotionDerived> & m) const
    {
      typedef typename MotionAlgebraAction<JointMotionSubspaceHelicalTpl,MotionDerived>::ReturnType ReturnType;
      ReturnType res;
      MotionRef<ReturnType> v(res);
      v = m.cross(AxisAngular() + AxisLinear() * m_h);
      return res;
    }
    
    bool isEqual(const JointMotionSubspaceHelicalTpl &) const { return true; }

    Scalar & pitch() { return m_h; }
    const Scalar & pitch() const { return m_h; }

    protected:
    Scalar m_h;
  }; // struct JointMotionSubspaceHelicalTpl

  template<typename _Scalar, int _Options, int _axis>
  struct JointHelicalTpl
  {
    typedef _Scalar Scalar;
    
    enum
    {
      Options = _Options,
      axis = _axis
    };
  };

  template<typename S1, int O1,typename S2, int O2, int axis>
  struct MultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceHelicalTpl<S2,O2,axis> >
  {
    typedef Eigen::Matrix<S2,6,1,O2> ReturnType;
  };

/* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceHelicalTpl<S2,O2,0> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceHelicalTpl<S2,O2,0> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & constraint)
      {
        ReturnType res;
        const S2 m_h = constraint.pitch();
        
        /* Y(:,3) = ( 0,-z, y,  I00+yy+zz,  I01-xy   ,  I02-xz   ) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        const typename Inertia::Symmetric3 & I = Y.inertia();
        
        res <<
        m*m_h,
        -m*z,
        m*y,
        I(0,0)+m*(y*y+z*z),
        I(0,1)-m*x*y,
        I(0,2)-m*x*z;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceHelicalTpl<S2,O2,1> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceHelicalTpl<S2,O2,1> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & constraint)
      {
        ReturnType res;
        const S2 m_h = constraint.pitch();
        
        /* Y(:,4) = ( z, 0,-x,  I10-xy   ,  I11+xx+zz,  I12-yz   ) */
        const S1
        &m = Y.mass(),
        &x = Y.lever()[0],
        &y = Y.lever()[1],
        &z = Y.lever()[2];
        const typename Inertia::Symmetric3 & I = Y.inertia();
        
        res <<
        m*z,
        m*m_h,
        -m*x,
        I(1,0)-m*x*y,
        I(1,1)+m*(x*x+z*z),
        I(1,2)-m*y*z;
        
        return res;
      }
    };
    
    template<typename S1, int O1, typename S2, int O2>
    struct LhsMultiplicationOp<InertiaTpl<S1,O1>, JointMotionSubspaceHelicalTpl<S2,O2,2> >
    {
      typedef InertiaTpl<S1,O1> Inertia;
      typedef JointMotionSubspaceHelicalTpl<S2,O2,2> Constraint;
      typedef typename MultiplicationOp<Inertia,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Inertia & Y,
                                   const Constraint & constraint)
      {
        ReturnType res;
        const S2 m_h = constraint.pitch();
        
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
        m*m_h,
        I(2,0)-m*x*z,
        I(2,1)-m*y*z,
        I(2,2)+m*(x*x+y*y);
        
        return res;
      }
    };
  } // namespace impl
  
  template<typename M6Like,typename S2, int O2, int axis>
  struct MultiplicationOp<Eigen::MatrixBase<M6Like>, JointMotionSubspaceHelicalTpl<S2,O2,axis> >
  {
    typedef typename M6Like::ConstColXpr ReturnType;
  };
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  namespace impl
  {
    template<typename M6Like, typename Scalar, int Options, int axis>
    struct LhsMultiplicationOp<Eigen::MatrixBase<M6Like>, JointMotionSubspaceHelicalTpl<Scalar,Options,axis> >
    {
      typedef JointMotionSubspaceHelicalTpl<Scalar,Options,axis> Constraint;
      typedef typename MultiplicationOp<Eigen::MatrixBase<M6Like>,Constraint>::ReturnType ReturnType;
      static inline ReturnType run(const Eigen::MatrixBase<M6Like> & Y,
                                   const Constraint & constraint)
      {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
        return Y.col(Inertia::ANGULAR + axis) + Y.col(Inertia::LINEAR + axis) * constraint.pitch();
      }
    };
  } // namespace impl

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointHelicalTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataHelicalTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelHelicalTpl<Scalar,Options,axis> JointModelDerived;
    typedef JointMotionSubspaceHelicalTpl<Scalar,Options,axis> Constraint_t;
    typedef TransformHelicalTpl<Scalar,Options,axis> Transformation_t;
    typedef MotionHelicalTpl<Scalar,Options,axis> Motion_t;
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
  struct traits< JointDataHelicalTpl<_Scalar,_Options,axis> >
  {
    typedef JointHelicalTpl<_Scalar,_Options,axis> JointDerived;
    typedef _Scalar Scalar;
  };
  
  template<typename _Scalar, int _Options, int axis>
  struct traits< JointModelHelicalTpl<_Scalar,_Options,axis> >
  {
    typedef JointHelicalTpl<_Scalar,_Options,axis> JointDerived;
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataHelicalTpl
  : public JointDataBase< JointDataHelicalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointHelicalTpl<_Scalar,_Options,axis> JointDerived;
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

    JointDataHelicalTpl()
    : joint_q(ConfigVector_t::Zero())
    , joint_v(TangentVector_t::Zero())
    , S((Scalar)1)
    , M((Scalar)0,(Scalar)1,(Scalar)1,(Scalar)1)
    , v((Scalar)0,(Scalar)0)
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
    
  }; // struct JointDataHelicalTpl
  
  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType< NewScalar, JointModelHelicalTpl<Scalar,Options,axis> >
  {
    typedef JointModelHelicalTpl<NewScalar,Options,axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelHelicalTpl
  : public JointModelBase< JointModelHelicalTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointHelicalTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);

    typedef JointModelBase<JointModelHelicalTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    JointModelHelicalTpl(const Scalar & pitch) : m_pitch(pitch) {}
    
    template<typename ConfigVector>
    EIGEN_DONT_INLINE
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      data.joint_q[0] = qs[idx_q()];
      Scalar ca,sa; SINCOS(data.joint_q[0],&sa,&ca);
      // TODO still passing redundant information
      data.M.setValues(sa,ca,data.joint_q[0],m_pitch);
      data.S.pitch() = m_pitch;
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
      data.v.pitch() = m_pitch;
    }
    
    template<typename VectorLike, typename Matrix6Like>
    void calc_aba(JointDataDerived & data,
                  const Eigen::MatrixBase<VectorLike> & armature,
                  const Eigen::MatrixBase<Matrix6Like> & I,
                  const bool update_I) const
    {
      // I is data.Yaba[i] - Articulated Body Inertia of the sub-tree
      data.U = I.col(Inertia::ANGULAR + axis) + m_pitch *  I.col(Inertia::LINEAR + axis);
      data.StU[0] = data.U(Inertia::ANGULAR + axis) + m_pitch * data.U(Inertia::LINEAR + axis) + armature[0];
      data.Dinv[0] = Scalar(1) / data.StU[0];
      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I).noalias() -= data.UDinv * data.U.transpose();
    }
    
    static std::string classname()
    {
      return std::string("JointModelH") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelHelicalTpl<NewScalar,Options,axis> cast() const
    {
      typedef JointModelHelicalTpl<NewScalar,Options,axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

    protected:
    Scalar m_pitch;
    
  }; // struct JointModelHelicalTpl

  typedef JointHelicalTpl<double,0,0> JointHX;
  typedef JointDataHelicalTpl<double,0,0> JointDataHX;
  typedef JointModelHelicalTpl<double,0,0> JointModelHX;

  // typedef JointHelicalTpl<double,0,1> JointHY;
  // typedef JointDataHelicalTpl<double,0,1> JointDataHY;
  // typedef JointModelHelicalTpl<double,0,1> JointModelHY;

  // typedef JointHelicalTpl<double,0,2> JointHZ;
  // typedef JointDataHelicalTpl<double,0,2> JointDataHZ;
  // typedef JointModelHelicalTpl<double,0,2> JointModelHZ;

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointModelHelicalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointModelHelicalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointDataHelicalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointDataHelicalTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_multibody_joint_helical_hpp__
