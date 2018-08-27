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

#ifndef __se3_joint_spherical_hpp__
#define __se3_joint_spherical_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  template<typename Scalar, int Options> struct MotionSpherical;
  
  template<typename _Scalar, int _Options>
  struct traits< MotionSpherical<_Scalar,_Options> >
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
  }; // traits MotionSpherical

  template<typename _Scalar, int _Options>
  struct MotionSpherical : MotionBase< MotionSpherical<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(MotionSpherical);

    MotionSpherical() : w (Motion::Vector3(NAN, NAN, NAN)) {}
    MotionSpherical(const Motion::Vector3 & w) : w (w)  {}

    Vector3 & operator() () { return w; }
    const Vector3 & operator() () const { return w; }

    operator MotionPlain() const
    {
      return MotionPlain(MotionPlain::Vector3::Zero(), w);
    }
    
    template<typename MotionDerived>
    void addTo(MotionDense<MotionDerived> & v) const
    {
      v.angular() += w;
    }
    
    Vector3 w;
  }; // struct MotionSpherical

  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionSpherical<S1,O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    return typename MotionDerived::MotionPlain(m2.linear(),m2.angular() + m1.w);
  }

  template<typename Scalar, int Options> struct ConstraintSphericalTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintSphericalTpl<_Scalar,_Options> >
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
  }; // struct traits struct ConstraintSphericalTpl

  template<typename _Scalar, int _Options>
  struct ConstraintSphericalTpl : public ConstraintBase< ConstraintSphericalTpl<_Scalar,_Options> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintSphericalTpl);
    enum { NV = 3 };
    typedef typename traits<ConstraintSphericalTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintSphericalTpl>::JointForce JointForce;
    typedef typename traits<ConstraintSphericalTpl>::DenseBase DenseBase;
    
    int nv_impl() const { return NV; }
    
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
      cross(m.translation(),m.rotation(),X_subspace.template block<3,3>(LINEAR,0));
      X_subspace.template block<3,3>(ANGULAR,0) = m.rotation();

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

  }; // struct ConstraintSphericalTpl

              
  template<typename Scalar, int Options, typename Vector3Like>
  MotionSpherical<Scalar,Options>
  operator*(const ConstraintSphericalTpl<Scalar,Options> &,
            const Eigen::MatrixBase<Vector3Like>& v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
    return MotionSpherical<Scalar,Options>(v);
  }

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionSpherical<S2,O2> & m2)
  {
    return typename MotionDerived::MotionPlain(m1.template linear().cross(m2.w),
                                               m1.template angular().cross(m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,3,O2>
  operator*(const InertiaTpl<S1,O1> & Y,
            const ConstraintSphericalTpl<S2,O2> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    Eigen::Matrix<S2,6,3,O2> M;
    //    M.block <3,3> (Inertia::LINEAR, 0) = - Y.mass () * skew(Y.lever ());
    M.template block<3,3>(Inertia::LINEAR,0) = alphaSkew(-Y.mass(), Y.lever());
    M.template block<3,3>(Inertia::ANGULAR,0) = (Y.inertia() - Symmetric3::AlphaSkewSquare(Y.mass(), Y.lever())).matrix();
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
  
  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintSphericalTpl<S1,O1> >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintSphericalTpl<S1,O1>,MotionDerived >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  }

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
    typedef MotionSpherical<Scalar,Options> Motion_t;
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
  struct traits< JointDataSphericalTpl<Scalar,Options> >
  { typedef JointSphericalTpl<Scalar,Options> JointDerived; };
  
  template<typename Scalar, int Options>
  struct traits< JointModelSphericalTpl<Scalar,Options> >
  { typedef JointSphericalTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataSphericalTpl : public JointDataBase< JointDataSphericalTpl<_Scalar,_Options> >
  {
    typedef JointSphericalTpl<_Scalar,_Options> JointDerived;
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

    JointDataSphericalTpl () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataSphericalTpl

  template<typename _Scalar, int _Options>
  struct JointModelSphericalTpl : public JointModelBase< JointModelSphericalTpl<_Scalar,_Options> >
  {
    typedef JointSphericalTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelSphericalTpl>::id;
    using JointModelBase<JointModelSphericalTpl>::idx_q;
    using JointModelBase<JointModelSphericalTpl>::idx_v;
    using JointModelBase<JointModelSphericalTpl>::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename ConfigVectorLike>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<ConfigVectorLike> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVectorLike);
      typedef typename Eigen::Quaternion<typename ConfigVectorLike::Scalar,EIGEN_PLAIN_TYPE(ConfigVectorLike)::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;

      ConstQuaternionMap quat(q_joint.derived().data());
      //assert(std::fabs(quat.coeffs().squaredNorm()-1.) <= sqrt(Eigen::NumTraits<typename V::Scalar>::epsilon())); TODO: check validity of the rhs precision
      assert(std::fabs(quat.coeffs().squaredNorm()-1.) <= 1e-4);
      
      M.rotation(quat.matrix());
      M.translation().setZero();
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      typedef typename Eigen::Quaternion<typename ConfigVector::Scalar,ConfigVector::Options> Quaternion;
      typedef Eigen::Map<const Quaternion> ConstQuaternionMap;
      
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type & q = qs.template segment<NQ>(idx_q());
      
      ConstQuaternionMap quat(q.data());
      data.M.rotation(quat.matrix());
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      calc(data,qs.derived());
      
      data.v() = vs.template segment<NV>(idx_v());
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      data.U = I.template block<6,3>(0,Inertia::ANGULAR);
      data.Dinv = I.template block<3,3>(Inertia::ANGULAR,Inertia::ANGULAR).inverse();
      data.UDinv.template middleRows<3>(Inertia::ANGULAR).setIdentity(); // can be put in data constructor
      data.UDinv.template middleRows<3>(Inertia::LINEAR) = data.U.template block<3,3>(Inertia::LINEAR, 0) * data.Dinv;
      
      if (update_I)
      {
        I.template block<3,3>(Inertia::LINEAR,Inertia::LINEAR)
        -= data.UDinv.template middleRows<3>(Inertia::LINEAR) * I.template block<3,3> (Inertia::ANGULAR, Inertia::LINEAR);
        I.template block<6,3>(0,Inertia::ANGULAR).setZero();
        I.template block<3,3>(Inertia::ANGULAR,Inertia::LINEAR).setZero();
      }
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelSpherical"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelSphericalTpl

} // namespace se3

#endif // ifndef __se3_joint_spherical_hpp__
