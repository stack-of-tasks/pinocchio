//
// Copyright (c) 2015-2017 CNRS
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

#include <stdexcept>

namespace se3
{
  
  template <typename _Scalar, int _Options>
  struct BiasSphericalTpl;
  
  template <typename _Scalar, int _Options>
  struct traits< BiasSphericalTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,_Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,_Options> Matrix6;
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
      ANGULAR = 3,
      Options = _Options
    };
  };
  
  template <typename _Scalar, int _Options>
  struct BiasSphericalTpl : MotionBase< BiasSphericalTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(BiasSphericalTpl);
    Vector3 c_J;
    
    BiasSphericalTpl ()  {c_J.fill (NAN);}
    //BiasSpherical (const Motion::Vector3 & c_J) c_J (c_J) {}
    
    operator Motion () const { return Motion (Motion::Vector3::Zero(), c_J); }
    
    Vector3 & operator() () { return c_J; }
    const Vector3 & operator() () const { return c_J; }
    
    template<typename D2>
    bool isEqual_impl(const MotionDense<D2> & other) const
    { return other.linear().isZero() && other.angular() == c_J; }
    
    template<typename D2>
    void addTo(MotionDense<D2> & other) const
    { other.angular() += c_J; }
  }; // struct BiasSphericalTpl
  
  template <typename S2, int O2>
  inline Motion operator+ (const Motion & v, const BiasSphericalTpl<S2,O2> & c) { return Motion (v.linear (), v.angular () + c ()); }
  template <typename S1, int O1>
  inline Motion operator+ (const BiasSphericalTpl<S1,O1> & c, const Motion & v) { return Motion (v.linear (), v.angular () + c ()); }
  
  template <typename _Scalar, int _Options>
  struct MotionSphericalTpl;
  
  template <typename _Scalar, int _Options>
  struct traits< MotionSphericalTpl<_Scalar,_Options> > : traits< BiasSphericalTpl<_Scalar,_Options> >
  {};
  
  template <typename _Scalar, int _Options>
  struct MotionSphericalTpl : MotionBase< BiasSphericalTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(MotionSphericalTpl);

    MotionSphericalTpl () : w(Vector3::Constant(NAN)) {}
    MotionSphericalTpl (const Vector3 & w) : w (w)  {}
    
    Vector3 & operator() () { return w; }
    const Vector3 & operator() () const { return w; }
    
    operator MotionPlain() const { return MotionPlain(MotionPlain::Vector3::Zero(), w); }
    
    operator Vector3() const { return w; }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v) const
    {
      v.angular() += w;
    }
    
    Vector3 w;
  }; // struct MotionSphericalTpl
  
  template <typename S1, int O1, typename S2, int O2>
  inline MotionSphericalTpl<S1,O1> operator+(const MotionSphericalTpl<S1,O1> & m,
                                             const BiasSphericalTpl<S2,O2> & c)
  { return MotionSphericalTpl<S1,O1>(m.w + c.c_J); }
  
  template <typename S1, int O1, typename M2>
  typename MotionDense<M2>::MotionPlain operator+(const MotionSphericalTpl<S1,O1> & m1,
                                                  const MotionDense<M2> & m2)
  {
    return MotionDense<M2>::MotionPlain(m2.linear(),m2.angular() + m1.w);
  }
  
  template <typename _Scalar, int _Options>
  struct ConstraintRotationalSubspaceTpl;
  template <typename _Scalar, int _Options>
  struct traits < struct ConstraintRotationalSubspaceTpl<_Scalar,_Options> >
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
  
  template <typename _Scalar, int _Options>
  struct ConstraintRotationalSubspaceTpl : public ConstraintBase< ConstraintRotationalSubspaceTpl<_Scalar,_Options> >
  {
    enum { NV = 3, Options = _Options };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<_Scalar,3,3,_Options> Matrix3;
    typedef Eigen::Matrix<_Scalar,3,1,_Options> Vector3;
    typedef Eigen::Matrix<_Scalar,6,3,_Options> ConstraintDense;
    typedef Eigen::Matrix<_Scalar,6,3,_Options> DenseBase;
    typedef MotionSphericalTpl<_Scalar,_Options> MotionSpherical;
    
    Matrix3 S_minimal;
    
    Motion operator* (const MotionSpherical & vj) const
    { return Motion (Motion::Vector3::Zero (), S_minimal * vj ()); }
    
    ConstraintRotationalSubspaceTpl () : S_minimal () { S_minimal.fill (NAN); }
    ConstraintRotationalSubspaceTpl (const Matrix3 & subspace) : S_minimal (subspace) {}
    
    Matrix3 & operator() () { return S_minimal; }
    const Matrix3 & operator() () const { return S_minimal; }
    
    int nv_impl() const { return NV; }
    
    struct ConstraintTranspose
    {
      const ConstraintRotationalSubspaceTpl & ref;
      ConstraintTranspose(const ConstraintRotationalSubspaceTpl & ref) : ref(ref) {}
      
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
        return ref.S_minimal.transpose () * F.template bottomRows<3> ();
      }
    }; // struct ConstraintTranspose
    
    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }
    
    DenseBase matrix_impl() const
    {
      ConstraintDense S;
      (S.template block <3,3> (Inertia::LINEAR, 0)).setZero ();
      S.template block <3,3> (Inertia::ANGULAR, 0) = S_minimal;
      return S;
    }
    
    //      const typename Eigen::ProductReturnType<
    //      const ConstraintDense,
    //      const Matrix3
    //      >::Type
    Eigen::Matrix <Scalar,6,3, Options>
    se3Action (const SE3 & m) const
    {
      //        Eigen::Matrix <Scalar,6,3,Options> X_subspace;
      //        X_subspace.template block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
      //        X_subspace.template block <3,3> (Motion::ANGULAR, 0) = m.rotation ();
      //
      //        return (X_subspace * S_minimal).eval();
      
      Eigen::Matrix <Scalar,6,3,Options> result;
      result.template block <3,3> (Motion::ANGULAR, 0) = m.rotation () * S_minimal;
      for (int k = 0; k < 3; ++k)
        result.template block <3,3> (Motion::LINEAR, 0).col(k) =
        m.translation ().cross(result.template block <3,3> (Motion::ANGULAR, 0).col(k));
      
      return result;
    }
    
    DenseBase motionAction(const Motion & m) const
    {
      const typename Motion::ConstLinearType v = m.linear();
      const typename Motion::ConstAngularType w = m.angular();
      
      DenseBase res;
      cross(v,S_minimal,res.template middleRows<3>(Motion::LINEAR));
      cross(w,S_minimal,res.template middleRows<3>(Motion::ANGULAR));
      
      return res;
    }
    
  }; // struct ConstraintRotationalSubspaceTpl
  
  template<typename S1, int O1, typename D>
  Motion operator* (const ConstraintRotationalSubspaceTpl<S1,O1> & S, const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    return Motion (Motion::Vector3::Zero (), S () * v);
  }
  
  template <typename _Scalar, int _Options>
  struct JointSphericalZYXTpl
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef SE3Tpl<Scalar,Options> SE3;
    
    typedef BiasSphericalTpl<_Scalar,_Options> BiasSpherical;
    typedef MotionSphericalTpl<_Scalar,_Options> MotionSpherical;
    typedef ConstraintRotationalSubspaceTpl<_Scalar,_Options> ConstraintRotationalSubspace;
    
  }; // struct JointSphericalZYX
  
  typedef JointSphericalZYXTpl<double,0> JointSphericalZYX;

  inline Motion operator^ (const Motion & m1, const JointSphericalZYX::MotionSpherical & m2)
  {
//    const Motion::Matrix3 m2_cross (skew (Motion::Vector3 (-m2.w)));
//    return Motion(m2_cross * m1.linear (), m2_cross * m1.angular ());
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */

  template <typename _Scalar, int _Options>
//  const typename Eigen::ProductReturnType<
//  Eigen::Matrix < _Scalar, 6, 3, _Options >,
//  Eigen::Matrix < _Scalar, 3, 3, _Options> 
//  >::Type
  Eigen::Matrix <_Scalar,6,3,_Options>
  operator* (const InertiaTpl<_Scalar,_Options> & Y,
             const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace & S)
  {
    Eigen::Matrix < _Scalar, 6, 3, _Options > M;
    M.template topRows<3>() = alphaSkew ( -Y.mass (),  Y.lever () );
    M.template bottomRows<3> () =  (Y.inertia () -
       typename Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ())).matrix();

    return (M * S.S_minimal).eval();
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  //  inline Eigen::Matrix<double,6,3>
  template <typename _Scalar, int _Options>
#if EIGEN_VERSION_AT_LEAST(3,2,90)
  const typename Eigen::Product<
  const Eigen::Block<const Inertia::Matrix6,6,3>,
  const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace::Matrix3
  >
#else
  const typename Eigen::ProductReturnType<
  const Eigen::Block<const Inertia::Matrix6,6,3>,
  const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace::Matrix3
  >::Type
#endif
  operator*(const typename InertiaTpl<_Scalar,_Options>::Matrix6 & Y,
            const typename JointSphericalZYXTpl<_Scalar,_Options>::ConstraintRotationalSubspace & S)
  {
    return Y.template middleCols<3>(Inertia::ANGULAR) * S.S_minimal;
  }
  
  inline Eigen::Matrix<double,6,3>
  operator*(const Inertia::Matrix6 & Y,
            const JointSphericalZYX::ConstraintRotationalSubspace & S)
  {
    typedef Eigen::Matrix<double,6,3> ReturnType;
    return ReturnType(Y.middleCols<3>(Inertia::ANGULAR) * S.S_minimal);
  }

  namespace internal
  {
    template<>
    struct SE3GroupAction<JointSphericalZYX::ConstraintRotationalSubspace >
    {
//      typedef const typename Eigen::ProductReturnType<
//      Eigen::Matrix <double,6,3,0>,
//      Eigen::Matrix <double,3,3,0>
//      >::Type Type;
      typedef Eigen::Matrix <double,6,3,0> ReturnType;
    };
    
    template<typename MotionDerived>
    struct MotionAlgebraAction<JointSphericalZYX::ConstraintRotationalSubspace,MotionDerived>
    {
      typedef Eigen::Matrix <double,6,3,0> ReturnType;
    };
  }

  template<>
  struct traits<JointSphericalZYX>
  {
    enum {
      NQ = 3,
      NV = 3
    };
    typedef double Scalar;
    typedef JointDataSphericalZYX JointDataDerived;
    typedef JointModelSphericalZYX JointModelDerived;
    typedef JointSphericalZYX::ConstraintRotationalSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef JointSphericalZYX::MotionSpherical Motion_t;
    typedef JointSphericalZYX::BiasSpherical Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };
  template<> struct traits<JointDataSphericalZYX> { typedef JointSphericalZYX JointDerived; };
  template<> struct traits<JointModelSphericalZYX> { typedef JointSphericalZYX JointDerived; };

  struct JointDataSphericalZYX : public JointDataBase<JointDataSphericalZYX>
  {
    typedef JointSphericalZYX JointDerived;
    SE3_JOINT_TYPEDEF;


    typedef Eigen::Matrix<Scalar,6,6> Matrix6;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataSphericalZYX () : M(1), U(), Dinv(), UDinv() {}
    
  }; // strcut JointDataSphericalZYX

  struct JointModelSphericalZYX : public JointModelBase<JointModelSphericalZYX>
  {
    typedef JointSphericalZYX JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelSphericalZYX>::id;
    using JointModelBase<JointModelSphericalZYX>::idx_q;
    using JointModelBase<JointModelSphericalZYX>::idx_v;
    using JointModelBase<JointModelSphericalZYX>::setIndexes;
    typedef Motion::Vector3 Vector3;

    JointDataDerived createData() const { return JointDataDerived(); }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());

      double c0,s0; SINCOS (q(0), &s0, &c0);
      double c1,s1; SINCOS (q(1), &s1, &c1);
      double c2,s2; SINCOS (q(2), &s2, &c2);

      data.M.rotation () << c0 * c1,
                c0 * s1 * s2 - s0 * c2,
                c0 * s1 * c2 + s0 * s2,
                s0 * c1,
                s0 * s1 * s2 + c0 * c2,
                s0 * s1 * c2 - c0 * s2,
                -s1,
                c1 * s2,
                c1 * c2;

      data.S.S_minimal <<  -s1, 0., 1., c1 * s2, c2, 0, c1 * c2, -s2, 0;
    }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NQ> (idx_v ());

      double c0,s0; SINCOS (q(0), &s0, &c0);
      double c1,s1; SINCOS (q(1), &s1, &c1);
      double c2,s2; SINCOS (q(2), &s2, &c2);

      data.M.rotation () << c0 * c1,
                c0 * s1 * s2 - s0 * c2,
                c0 * s1 * c2 + s0 * s2,
                s0 * c1,
                s0 * s1 * s2 + c0 * c2,
                s0 * s1 * c2 - c0 * s2,
                -s1,
                c1 * s2,
                c1 * c2;


      data.S.S_minimal <<  -s1, 0., 1., c1 * s2, c2, 0, c1 * c2, -s2, 0;

      data.v () = data.S.S_minimal * q_dot;

      data.c ()(0) = -c1 * q_dot (0) * q_dot (1);
      data.c ()(1) = -s1 * s2 * q_dot (0) * q_dot (1) + c1 * c2 * q_dot (0) * q_dot (2) - s2 * q_dot (1) * q_dot (2);
      data.c ()(2) = -s1 * c2 * q_dot (0) * q_dot (1) - c1 * s2 * q_dot (0) * q_dot (2) - c2 * q_dot (1) * q_dot (2);
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.middleCols<3> (Inertia::ANGULAR) * data.S.S_minimal;
      Inertia::Matrix3 tmp (data.S.S_minimal.transpose() * data.U.middleRows<3> (Inertia::ANGULAR));
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

  }; // struct JointModelSphericalZYX

} // namespace se3

#endif // ifndef __se3_joint_spherical_ZYX_hpp__
