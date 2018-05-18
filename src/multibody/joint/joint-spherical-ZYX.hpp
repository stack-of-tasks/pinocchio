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

#ifndef __se3_joint_spherical_ZYX_hpp__
#define __se3_joint_spherical_ZYX_hpp__
#include <iostream>
#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{
  
  template<typename Scalar, int Options> struct BiasSphericalZYXTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< BiasSphericalZYXTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Matrix6 ActionMatrixType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type LinearType;
    typedef typename Vector6::template FixedSegmentReturnType<3>::Type AngularType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstLinearType;
    typedef typename Vector6::template ConstFixedSegmentReturnType<3>::Type ConstAngularType;
    typedef MotionTpl<Scalar,_Options> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  };
  
  template <typename _Scalar, int _Options>
  struct BiasSphericalZYXTpl : MotionBase< BiasSphericalZYXTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(BiasSphericalZYXTpl);
    
    BiasSphericalZYXTpl () : c_J(Vector3::Constant(NAN)) {}
    
    operator MotionPlain () const
    { return MotionPlain(MotionPlain::Vector3::Zero(),c_J); }
    
    Vector3 & operator() () { return c_J; }
    const Vector3 & operator() () const { return c_J; }
    
    template<typename D2>
    bool isEqual_impl(const MotionDense<D2> & other) const
    { return other.linear().isZero() && other.angular() == c_J; }
    
    template<typename D2>
    void addTo(MotionDense<D2> & other) const
    { other.angular() += c_J; }
    
    Vector3 c_J;
  }; // struct BiasSphericalZYXTpl
  
  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionDense<MotionDerived> & v, const BiasSphericalZYXTpl<S2,O2> & c)
  { return typename MotionDerived::MotionPlain(v.linear(), v.angular() + c()); }
  
  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const BiasSphericalZYXTpl<S1,O1> & c, const MotionDense<MotionDerived> & v)
  { return typename MotionDerived::MotionPlain(v.linear(), v.angular() + c()); }
  
  template<typename Scalar, int Options> struct MotionSphericalZYXTpl;
  
  template<typename Scalar, int Options>
  struct traits< MotionSphericalZYXTpl<Scalar,Options> >
  : traits< BiasSphericalZYXTpl<Scalar,Options> >
  {};
  
  template<typename _Scalar, int _Options>
  struct MotionSphericalZYXTpl : MotionBase< BiasSphericalZYXTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(MotionSphericalZYXTpl);

    MotionSphericalZYXTpl () : w(Vector3::Constant(NAN)) {}
    
    template<typename Vector3Like>
    MotionSphericalZYXTpl(const Eigen::MatrixBase<Vector3Like> & w) : w (w)
    {}
    
    Vector3 & operator() () { return w; }
    const Vector3 & operator() () const { return w; }
    
    operator MotionPlain() const
    { return MotionPlain(MotionPlain::Vector3::Zero(),w); }
    
    operator Vector3() const { return w; }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v) const
    {
      v.angular() += w;
    }
    
    Vector3 w;
  }; // struct MotionSphericalZYXTpl
  
  template <typename S1, int O1, typename S2, int O2>
  inline MotionSphericalZYXTpl<S1,O1>
  operator+(const MotionSphericalZYXTpl<S1,O1> & m,
            const BiasSphericalZYXTpl<S2,O2> & c)
  { return MotionSphericalZYXTpl<S1,O1>(m.w + c.c_J); }
  
  template <typename S1, int O1, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionSphericalZYXTpl<S1,O1> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typedef typename MotionDerived::MotionPlain ReturnType;
    return ReturnType(m2.linear(),m2.angular()+ m1.w);
  }
  
  template<typename Scalar, int Options> struct ConstraintSphericalZYXTpl;
  
  template <typename _Scalar, int _Options>
  struct traits< ConstraintSphericalZYXTpl<_Scalar,_Options> >
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
    typedef Eigen::Matrix<Scalar,3,1,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,3,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,3,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // struct traits struct ConstraintRotationalSubspace
  
  template<typename _Scalar, int _Options>
  struct ConstraintSphericalZYXTpl : public ConstraintBase< ConstraintSphericalZYXTpl<_Scalar,_Options> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintSphericalZYXTpl);
    enum { NV = 3, Options = _Options };
    
    typedef typename traits<ConstraintSphericalZYXTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintSphericalZYXTpl>::JointForce JointForce;
    typedef typename traits<ConstraintSphericalZYXTpl>::DenseBase DenseBase;
    
    ConstraintSphericalZYXTpl()
    : S_minimal(Matrix3::Constant(NAN))
    {}
    
    template<typename Matrix3Like>
    ConstraintSphericalZYXTpl(const Eigen::MatrixBase<Matrix3Like> & subspace)
    : S_minimal(subspace)
    {  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like,3,3); }
    
    template<typename S1, int O1>
    Motion operator*(const MotionSphericalZYXTpl<S1,O1> & vj) const
    { return Motion(Motion::Vector3::Zero(),
                    S_minimal * vj());
      
    }
    template<typename Vector3Like>
    Motion operator*(const Eigen::MatrixBase<Vector3Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return Motion(Motion::Vector3::Zero(), S_minimal * v);
    }
    
    
    Matrix3 & operator() () { return S_minimal; }
    const Matrix3 & operator() () const { return S_minimal; }
    
    int nv_impl() const { return NV; }
    
    struct ConstraintTranspose
    {
      const ConstraintSphericalZYXTpl & ref;
      ConstraintTranspose(const ConstraintSphericalZYXTpl & ref) : ref(ref) {}
      
      template<typename Derived>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      const typename Eigen::Product<
      Eigen::Transpose<const Matrix3>,
      Eigen::Block<const typename ForceDense<Derived>::Vector6,3,1>
      >
#else
      const typename Eigen::ProductReturnType<
      Eigen::Transpose<const Matrix3>,
      //        typename Motion::ConstAngular_t::Base /* This feature leads currently to a bug */
      Eigen::Block<const typename ForceDense<Derived>::Vector6,3,1>
      >::Type
#endif
      operator* (const ForceDense<Derived> & phi) const
      {
        return ref.S_minimal.transpose() * phi.angular();
      }
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      const typename Eigen::Product<
      typename Eigen::Transpose<const Matrix3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >
#else
      const typename Eigen::ProductReturnType<
      typename Eigen::Transpose<const Matrix3>,
      typename Eigen::MatrixBase<const D>::template NRowsBlockXpr<3>::Type
      >::Type
#endif
      operator* (const Eigen::MatrixBase<D> & F) const
      {
        EIGEN_STATIC_ASSERT(D::RowsAtCompileTime==6,THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE)
        return ref.S_minimal.transpose () * F.template middleRows<3>(ANGULAR);
      }
    }; // struct ConstraintTranspose
    
    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }
    
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template middleRows<3>(LINEAR).setZero();
      S.template middleRows<3>(ANGULAR) = S_minimal;
      return S;
    }
    
    //      const typename Eigen::ProductReturnType<
    //      const ConstraintDense,
    //      const Matrix3
    //      >::Type
    template<typename S1, int O1>
    Eigen::Matrix<Scalar,6,3,Options>
    se3Action(const SE3Tpl<S1,O1> & m) const
    {
      //        Eigen::Matrix <Scalar,6,3,Options> X_subspace;
      //        X_subspace.template block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
      //        X_subspace.template block <3,3> (Motion::ANGULAR, 0) = m.rotation ();
      //
      //        return (X_subspace * S_minimal).eval();
      
      Eigen::Matrix<Scalar,6,3,Options> result;
      result.template middleRows<3>(ANGULAR).noalias() = m.rotation () * S_minimal;
      for(int k = 0; k < 3; ++k)
        result.template middleRows<3>(LINEAR).col(k) =
        m.translation().cross(result.template middleRows<3>(Motion::ANGULAR).col(k));
      
      return result;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();
      
      DenseBase res;
      cross(v,S_minimal,res.template middleRows<3>(LINEAR));
      cross(w,S_minimal,res.template middleRows<3>(ANGULAR));
      
      return res;
    }
    
    // data
    Matrix3 S_minimal;
    
  }; // struct ConstraintSphericalZYXTpl
  
//  template <typename _Scalar, int _Options>
//  struct JointSphericalZYXTpl
//  {
//    typedef _Scalar Scalar;
//    enum { Options = _Options };
//    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
//    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
//    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
//    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
//    typedef MotionTpl<Scalar,Options> Motion;
//    typedef ForceTpl<Scalar,Options> Force;
//    typedef SE3Tpl<Scalar,Options> SE3;
//
//    typedef BiasSphericalZYXTpl<_Scalar,_Options> BiasSpherical;
//    typedef MotionSphericalZYXTpl<_Scalar,_Options> MotionSpherical;
//    typedef ConstraintSphericalZYXTpl<_Scalar,_Options> ConstraintRotationalSubspace;
//
//  }; // struct JointSphericalZYX
  
//  typedef JointSphericalZYXTpl<double,0> JointSphericalZYX;

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionSphericalZYXTpl<S2,O2> & m2)
  {
//    const Motion::Matrix3 m2_cross (skew (Motion::Vector3 (-m2.w)));
//    return Motion(m2_cross * m1.linear (), m2_cross * m1.angular ());
    typedef typename MotionDerived::MotionPlain ReturnType;
    return ReturnType(m1.linear().cross(m2.w), m1.angular().cross(m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template <typename S1, int O1, typename S2, int O2>
  Eigen::Matrix<S1,6,3,O1>
  operator*(const InertiaTpl<S1,O1> & Y,
            const ConstraintSphericalZYXTpl<S2,O2> & S)
  {
    typedef typename InertiaTpl<S1,O1>::Symmetric3 Symmetric3;
    typedef ConstraintSphericalZYXTpl<S2,O2> Constraint;
    Eigen::Matrix<S1,6,3,O1> M;
    alphaSkew (-Y.mass(),Y.lever(),M.template middleRows<3>(Constraint::LINEAR));
    M.template middleRows<3>(Constraint::ANGULAR) =  (Y.inertia () -
    typename Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ())).matrix();

    return (M * S.S_minimal).eval();
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  //  inline Eigen::Matrix<double,6,3>
  template<typename Matrix6Like, typename S2, int O2>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
  const typename Eigen::Product<
  typename Eigen::internal::remove_const<typename SizeDepType<3>::ColsReturn<Matrix6Like>::ConstType>::type,
  typename ConstraintSphericalZYXTpl<S2,O2>::Matrix3
  >
#else
  const typename Eigen::ProductReturnType<
  typename Eigen::internal::remove_const<typename SizeDepType<3>::ColsReturn<Matrix6Like>::ConstType>::type,
  typename ConstraintSphericalZYXTpl<S2,O2>::Matrix3
  >::Type
#endif
  operator*(const Eigen::MatrixBase<Matrix6Like> & Y,
            const ConstraintSphericalZYXTpl<S2,O2> & S)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix6Like,6,6);
    return Y.derived().template middleCols<3>(Inertia::ANGULAR) * S.S_minimal;
  }

  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintSphericalZYXTpl<S1,O1> >
    {
//      typedef const typename Eigen::ProductReturnType<
//      Eigen::Matrix <double,6,3,0>,
//      Eigen::Matrix <double,3,3,0>
//      >::Type Type;
      typedef Eigen::Matrix<S1,6,3,O1> ReturnType;
    };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintSphericalZYXTpl<S1,O1>, MotionDerived >
    {
      typedef Eigen::Matrix<S1,6,3,O1> ReturnType;
    };
  }

  template<typename Scalar, int Options> struct JointSphericalZYXTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointSphericalZYXTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 3,
      NV = 3
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataSphericalZYXTpl<Scalar,Options> JointDataDerived;
    typedef JointModelSphericalZYXTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintSphericalZYXTpl<Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionSphericalZYXTpl<Scalar,Options> Motion_t;
    typedef BiasSphericalZYXTpl<Scalar,Options> Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options>
  struct traits< JointDataSphericalZYXTpl<Scalar,Options> >
  { typedef JointSphericalZYXTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits< JointModelSphericalZYXTpl<Scalar,Options> >
  { typedef JointSphericalZYXTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataSphericalZYXTpl : public JointDataBase< JointDataSphericalZYXTpl<_Scalar,_Options> >
  {
    typedef JointSphericalZYXTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataSphericalZYXTpl () : M(1), U(), Dinv(), UDinv() {}
    
  }; // strcut JointDataSphericalZYXTpl

  template<typename _Scalar, int _Options>
  struct JointModelSphericalZYXTpl : public JointModelBase< JointModelSphericalZYXTpl<_Scalar,_Options> >
  {
    typedef JointSphericalZYXTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelSphericalZYXTpl>::id;
    using JointModelBase<JointModelSphericalZYXTpl>::idx_q;
    using JointModelBase<JointModelSphericalZYXTpl>::idx_v;
    using JointModelBase<JointModelSphericalZYXTpl>::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type & q = qs.template segment<NQ>(idx_q());
      
      typedef typename ConfigVector::Scalar S2;

      S2 c0,s0; SINCOS(q(0), &s0, &c0);
      S2 c1,s1; SINCOS(q(1), &s1, &c1);
      S2 c2,s2; SINCOS(q(2), &s2, &c2);

      data.M.rotation () << c0 * c1,
                c0 * s1 * s2 - s0 * c2,
                c0 * s1 * c2 + s0 * s2,
                s0 * c1,
                s0 * s1 * s2 + c0 * c2,
                s0 * s1 * c2 - c0 * s2,
                -s1,
                c1 * s2,
                c1 * c2;

      data.S.S_minimal
      <<  -s1, Scalar(0), Scalar(1),
      c1 * s2, c2, Scalar(0),
      c1 * c2, -s2, Scalar(0);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type & q = qs.template segment<NQ>(idx_q());
      
      typedef typename ConfigVector::Scalar S2;
      
      S2 c0,s0; SINCOS(q(0), &s0, &c0);
      S2 c1,s1; SINCOS(q(1), &s1, &c1);
      S2 c2,s2; SINCOS(q(2), &s2, &c2);
      
      data.M.rotation () << c0 * c1,
          c0 * s1 * s2 - s0 * c2,
          c0 * s1 * c2 + s0 * s2,
          s0 * c1,
          s0 * s1 * s2 + c0 * c2,
          s0 * s1 * c2 - c0 * s2,
          -s1,
          c1 * s2,
          c1 * c2;
      
      data.S.S_minimal
      <<  -s1, Scalar(0), Scalar(1),
      c1 * s2, c2, Scalar(0),
      c1 * c2, -s2, Scalar(0);
    
      typename TangentVector::template ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.template segment<NV>(idx_v());

      data.v().noalias() = data.S.S_minimal * q_dot;

      data.c()(0) = -c1 * q_dot(0) * q_dot(1);
      data.c()(1) = -s1 * s2 * q_dot(0) * q_dot(1) + c1 * c2 * q_dot(0) * q_dot(2) - s2 * q_dot(1) * q_dot(2);
      data.c()(2) = -s1 * c2 * q_dot(0) * q_dot(1) - c1 * s2 * q_dot(0) * q_dot(2) - c2 * q_dot(1) * q_dot(2);
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      typedef Eigen::Matrix<S2,3,3,O2> Matrix3;
      
      data.U.noalias() = I.template middleCols<3>(Motion::ANGULAR) * data.S.S_minimal;
      Matrix3 tmp(data.S.S_minimal.transpose() * data.U.template middleRows<3>(Motion::ANGULAR));
      data.Dinv = tmp.inverse();
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelSphericalZYX"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelSphericalZYXTpl

} // namespace se3

#endif // ifndef __se3_joint_spherical_ZYX_hpp__
