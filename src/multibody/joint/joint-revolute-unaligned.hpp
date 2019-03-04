//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_joint_revolute_unaligned_hpp__
#define __pinocchio_joint_revolute_unaligned_hpp__

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options=0> struct MotionRevoluteUnalignedTpl;
  typedef MotionRevoluteUnalignedTpl<double> MotionRevoluteUnaligned;
  
  namespace internal
  {
    template<typename Scalar, int Options>
    struct SE3GroupAction< MotionRevoluteUnalignedTpl<Scalar,Options> >
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< MotionRevoluteUnalignedTpl<Scalar,Options>, MotionDerived>
    {
      typedef MotionTpl<Scalar,Options> ReturnType;
    };
  }

  template<typename _Scalar, int _Options>
  struct traits< MotionRevoluteUnalignedTpl<_Scalar,_Options> >
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
  }; // traits MotionRevoluteUnalignedTpl

  template<typename _Scalar, int _Options>
  struct MotionRevoluteUnalignedTpl : MotionBase< MotionRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MOTION_TYPEDEF_TPL(MotionRevoluteUnalignedTpl);

    MotionRevoluteUnalignedTpl() {}
    
    template<typename Vector3Like, typename OtherScalar>
    MotionRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis,
                               const OtherScalar & w)
    : axis(axis)
    , w(w)
    {}

//    operator MotionPlain() const
//    { 
//      return MotionPlain(MotionPlain::Vector3::Zero(),
//                         axis*w);
//    }
    
    template<typename MotionDerived>
    void addTo(MotionDense<MotionDerived> & v) const
    {
      v.angular() += axis*w;
    }
    
    template<typename Derived>
    void setTo(MotionDense<Derived> & other) const
    {
      other.linear().setZero();
      other.angular().noalias() = axis*w;
    }

    template<typename S2, int O2, typename D2>
    void se3Action_impl(const SE3Tpl<S2,O2> & m, MotionDense<D2> & v) const
    {
      // Angular
      v.angular().noalias() = w * m.rotation() * axis;

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
      v3_tmp.noalias() = axis.cross(m.translation());
      v3_tmp *= w;
      v.linear().noalias() = m.rotation().transpose() * v3_tmp;
      
      // Angular
      v.angular().noalias() = m.rotation().transpose() * axis;
      v.angular() *= w;
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
      mout.linear().noalias() = v.linear().cross(axis);
      mout.linear() *= w;
      
      // Angular
      mout.angular().noalias() = v.angular().cross(axis);
      mout.angular() *= w;
    }
    
    template<typename M1>
    MotionPlain motionAction(const MotionDense<M1> & v) const
    {
      MotionPlain res;
      motionAction(v,res);
      return res;
    }
    
    // data
    Vector3 axis;
    Scalar w;
    
  }; // struct MotionRevoluteUnalignedTpl

  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionRevoluteUnalignedTpl<S1,O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<typename Scalar, int Options> struct ConstraintRevoluteUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintRevoluteUnalignedTpl<_Scalar,_Options> >
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
    typedef MotionRevoluteUnalignedTpl<Scalar,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintRevoluteUnalignedTpl

  template<typename _Scalar, int _Options>
  struct ConstraintRevoluteUnalignedTpl
  : ConstraintBase< ConstraintRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRevoluteUnalignedTpl);
    enum { NV = 1, Options = _Options };
    
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::JointForce JointForce;
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::DenseBase DenseBase;
    
    ConstraintRevoluteUnalignedTpl() {}
    
    template<typename Vector3Like>
    ConstraintRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {}
    
    template<typename Vector1Like>
    JointMotion __mult__(const Eigen::MatrixBase<Vector1Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector1Like,1);
      return JointMotion(axis,v[0]);
    }
    
    template<typename S1, int O1>
    Eigen::Matrix<Scalar,6,1,Options> se3Action(const SE3Tpl<S1,O1> & m) const
    {
      /* X*S = [ R pxR ; 0 R ] [ 0 ; a ] = [ px(Ra) ; Ra ] */
      Eigen::Matrix<Scalar,6,1,Options> res;
      res.template segment<3>(ANGULAR).noalias() = m.rotation() * axis;
      res.template segment<3>(LINEAR).noalias() = m.translation().cross(res.template segment<3>(ANGULAR));
      return res;
    }
    
    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      const ConstraintRevoluteUnalignedTpl & ref;
      TransposeConst(const ConstraintRevoluteUnalignedTpl & ref) : ref(ref) {}
      
      template<typename Derived>
      Eigen::Matrix<
      typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstAngularType),
      1,1>
      operator*(const ForceDense<Derived> & f) const
      {
        typedef Eigen::Matrix<
        typename PINOCCHIO_EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstAngularType),
        1,1> ReturnType;
        
        ReturnType res; res[0] = ref.axis.dot(f.angular());
        return res;
      }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      const Eigen::Product<
      Eigen::Transpose<const Vector3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >
#else
      const typename Eigen::ProductReturnType<
      Eigen::Transpose<const Vector3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >::Type
#endif
      operator*(const Eigen::MatrixBase<D> & F)
      {
        EIGEN_STATIC_ASSERT(D::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        /* Return ax.T * F[3:end,:] */
        return ref.axis.transpose() * F.template middleRows<3>(ANGULAR);
      }
      
    };
    
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
      S << Vector3::Zero(), axis;
      return S;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();
      
      DenseBase res;
      res << v.cross(axis), w.cross(axis);
      
      return res;
    }
    
    // data
    Vector3 axis;
    
  }; // struct ConstraintRevoluteUnalignedTpl
  
  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteUnalignedTpl<S2,O2> & m2)
  {
    /* m1xm2 = [ v1xw2 + w1xv2; w1xw2 ] = [ v1xw2; w1xw2 ] */
    return m2.motionAction(m1);
  }
  
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,1,O2>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintRevoluteUnalignedTpl<S2,O2> & cru)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    /* YS = [ m -mcx ; mcx I-mcxcx ] [ 0 ; w ] = [ mcxw ; Iw -mcxcxw ] */
    const typename Inertia::Scalar & m                 = Y.mass();
    const typename Inertia::Vector3 & c      = Y.lever();
    const typename Inertia::Symmetric3 & I   = Y.inertia();
    
    Eigen::Matrix<S2,6,1,O2>res;
    res.template segment<3>(Inertia::LINEAR) = -m*c.cross(cru.axis);
    res.template segment<3>(Inertia::ANGULAR).noalias() = I*cru.axis;
    res.template segment<3>(Inertia::ANGULAR) += c.cross(res.template segment<3>(Inertia::LINEAR));
    return res;
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  
  template<typename M6Like, typename S2, int O2>
  inline
#if EIGEN_VERSION_AT_LEAST(3,2,90)
  const typename Eigen::Product<
  typename Eigen::internal::remove_const<typename SizeDepType<3>::ColsReturn<M6Like>::ConstType>::type,
  typename ConstraintRevoluteUnalignedTpl<S2,O2>::Vector3
  >
#else
  const typename Eigen::ProductReturnType<
  typename Eigen::internal::remove_const<typename SizeDepType<3>::ColsReturn<M6Like>::ConstType>::type,
  typename ConstraintRevoluteUnalignedTpl<S2,O2>::Vector3
  >::Type
#endif
  operator*(const Eigen::MatrixBase<M6Like> & Y, const ConstraintRevoluteUnalignedTpl<S2,O2> & cru)
  {
    typedef ConstraintRevoluteUnalignedTpl<S2,O2> Constraint;
    return Y.derived().template middleCols<3>(Constraint::ANGULAR) * cru.axis;
  }
  
  namespace internal
  {
    template<typename Scalar, int Options>
    struct SE3GroupAction< ConstraintRevoluteUnalignedTpl<Scalar,Options> >
    { typedef Eigen::Matrix<Scalar,6,1,Options>  ReturnType; };
    
    template<typename Scalar, int Options, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintRevoluteUnalignedTpl<Scalar,Options>,MotionDerived >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  }

  template<typename Scalar, int Options> struct JointRevoluteUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataRevoluteUnalignedTpl<Scalar,Options> JointDataDerived;
    typedef JointModelRevoluteUnalignedTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintRevoluteUnalignedTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionRevoluteUnalignedTpl<Scalar,Options> Motion_t;
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
  struct traits< JointDataRevoluteUnalignedTpl<Scalar,Options> >
  { typedef JointRevoluteUnalignedTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits <JointModelRevoluteUnalignedTpl<Scalar,Options> >
  { typedef JointRevoluteUnalignedTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataRevoluteUnalignedTpl
  : public JointDataBase< JointDataRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnalignedTpl() {}
    
    template<typename Vector3Like>
    JointDataRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : M(1)
    , S(axis)
    , v(axis,(Scalar)NAN)
    , U(), Dinv(), UDinv()
    {}

    static std::string classname() { return std::string("JointDataRevoluteUnaligned"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataRevoluteUnalignedTpl

  PINOCCHIO_JOINT_CAST_TYPE_SPECIALIZATION(JointModelRevoluteUnalignedTpl);
  template<typename _Scalar, int _Options>
  struct JointModelRevoluteUnalignedTpl
  : public JointModelBase< JointModelRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnalignedTpl<_Scalar,_Options> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE;
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    
    typedef JointModelBase<JointModelRevoluteUnalignedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointModelRevoluteUnalignedTpl() {}
    
    JointModelRevoluteUnalignedTpl(const Scalar & x, const Scalar & y, const Scalar & z)
    : axis(x,y,z)
    {
      axis.normalize();
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }
    
    template<typename Vector3Like>
    JointModelRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Vector3Like);
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    
    using Base::isEqual;
    bool isEqual(const JointModelRevoluteUnalignedTpl & other) const
    {
      return Base::isEqual(other) && axis == other.axis;
    }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar OtherScalar;
      typedef Eigen::AngleAxis<Scalar> AngleAxis;
      
      const OtherScalar & q = qs[idx_q()];
      
      data.M.rotation(AngleAxis(q,axis).toRotationMatrix());
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());

      data.v.w = (Scalar)vs[idx_v()];
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis;
      data.Dinv[0] = (Scalar)(1)/axis.dot(data.U.template segment<3>(Motion::ANGULAR));
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using math::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelRevoluteUnaligned"); }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnalignedTpl<NewScalar,Options> cast() const
    {
      typedef JointModelRevoluteUnalignedTpl<NewScalar,Options> ReturnType;
      ReturnType res(axis.template cast<NewScalar>());
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

    // data
    
    ///
    /// \brief 3d main axis of the joint.
    ///
    Vector3 axis;
  }; // struct JointModelRevoluteUnalignedTpl

} //namespace pinocchio

#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_constructor< ::pinocchio::JointDataRevoluteUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options>
  struct has_nothrow_copy< ::pinocchio::JointDataRevoluteUnalignedTpl<Scalar,Options> >
  : public integral_constant<bool,true> {};
}


#endif // ifndef __pinocchio_joint_revolute_unaligned_hpp__
