//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_joint_revolute_unaligned_hpp__
#define __se3_joint_revolute_unaligned_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace se3
{

  template<typename Scalar, int Options> struct MotionRevoluteUnalignedTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< MotionRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
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
    MOTION_TYPEDEF_TPL(MotionRevoluteUnalignedTpl);

    MotionRevoluteUnalignedTpl() : axis(Vector3::Constant(NAN)), w(NAN) {}
    
    template<typename Vector3Like, typename OtherScalar>
    MotionRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis,
                               const OtherScalar & w)
    : axis(axis), w(w)
    {}

    operator MotionPlain() const
    { 
      return MotionPlain(MotionPlain::Vector3::Zero(),
                         axis*w);
    }
    
    template<typename MotionDerived>
    void addTo(MotionDense<MotionDerived> & v) const
    {
      v.angular() += axis*w;
    }
    
    // data
    Vector3 axis;
    double w;
    
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
    typedef Eigen::Matrix<Scalar,1,1,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintRevoluteUnalignedTpl

  template<typename _Scalar, int _Options>
  struct ConstraintRevoluteUnalignedTpl : ConstraintBase< ConstraintRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRevoluteUnalignedTpl);
    enum { NV = 1, Options = _Options };
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::JointForce JointForce;
    typedef typename traits<ConstraintRevoluteUnalignedTpl>::DenseBase DenseBase;
    
    ConstraintRevoluteUnalignedTpl() : axis(Vector3::Constant(NAN)) {}
    
    template<typename Vector3Like>
    ConstraintRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {}
    
    template<typename D>
    MotionRevoluteUnalignedTpl<Scalar,Options>
    operator*(const Eigen::MatrixBase<D> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,1);
      return MotionRevoluteUnalignedTpl<Scalar,Options>(axis,v[0]);
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
      typename EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstAngularType),
      1,1>
      operator*(const ForceDense<Derived> & f) const
      {
        typedef Eigen::Matrix<
        typename EIGEN_DOT_PRODUCT_RETURN_TYPE(Vector3,typename ForceDense<Derived>::ConstAngularType),
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
    typedef typename MotionDerived::MotionPlain ReturnType;
    const typename MotionDerived::ConstLinearType v1 = m1.linear();
    const typename MotionDerived::ConstAngularType w1 = m1.angular();
    const typename ReturnType::Vector3 w2(m2.axis * m2.w);
    return ReturnType(v1.cross(w2),w1.cross(w2));
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
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
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
  struct JointDataRevoluteUnalignedTpl : public JointDataBase< JointDataRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    typedef JointRevoluteUnalignedTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnalignedTpl()
      : M(1),S(Eigen::Vector3d::Constant(NAN)),v(Eigen::Vector3d::Constant(NAN),NAN)
      , U(), Dinv(), UDinv()
    {}
    
    JointDataRevoluteUnalignedTpl(const Motion::Vector3 & axis)
      : M(1),S(axis),v(axis,NAN)
      , U(), Dinv(), UDinv()
    {}

  }; // struct JointDataRevoluteUnalignedTpl

  template<typename _Scalar, int _Options>
  struct JointModelRevoluteUnalignedTpl : public JointModelBase< JointModelRevoluteUnalignedTpl<_Scalar,_Options> >
  {
    typedef JointRevoluteUnalignedTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;
    typedef Eigen::Matrix<_Scalar,3,1,_Options> Vector3;

    using JointModelBase<JointModelRevoluteUnalignedTpl>::id;
    using JointModelBase<JointModelRevoluteUnalignedTpl>::idx_q;
    using JointModelBase<JointModelRevoluteUnalignedTpl>::idx_v;
    using JointModelBase<JointModelRevoluteUnalignedTpl>::setIndexes;
    
    JointModelRevoluteUnalignedTpl() : axis(Eigen::Vector3d::Constant(NAN))   {}
    
    template<typename OtherScalar>
    JointModelRevoluteUnalignedTpl(const OtherScalar x, const OtherScalar y, const OtherScalar z)
    {
      axis << x, y, z ;
      axis.normalize();
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }
    
    template<typename Vector3Like>
    JointModelRevoluteUnalignedTpl(const Eigen::MatrixBase<Vector3Like> & axis)
    : axis(axis)
    {
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointDataDerived createData() const { return JointDataDerived(axis); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
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
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      calc(data,qs.derived());

      data.v.w = (Scalar)vs[idx_v()];
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * axis;
      data.Dinv[0] = (Scalar)(1)/axis.dot(data.U.template segment<3>(Motion::ANGULAR));
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelRevoluteUnaligned"); }
    std::string shortname() const { return classname(); }

    Vector3 axis;
  }; // struct JointModelRevoluteUnalignedTpl

} //namespace se3


#endif // ifndef __se3_joint_revolute_unaligned_hpp__
