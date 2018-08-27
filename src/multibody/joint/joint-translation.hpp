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

#ifndef __se3_joint_translation_hpp__
#define __se3_joint_translation_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  template<typename Scalar, int Options> struct MotionTranslationTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< MotionTranslationTpl<_Scalar,_Options> >
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
  }; // traits MotionTranslationTpl

  template<typename _Scalar, int _Options>
  struct MotionTranslationTpl : MotionBase< MotionTranslationTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(MotionTranslationTpl);

    MotionTranslationTpl()                   : v (Motion::Vector3 (NAN, NAN, NAN)) {}
    template<typename Vector3Like>
    MotionTranslationTpl(const Eigen::MatrixBase<Vector3Like> & v) : v (v)  {}
    
    MotionTranslationTpl(const MotionTranslationTpl & other) : v (other.v)  {}
 
    Vector3 & operator()() { return v; }
    const Vector3 & operator()() const { return v; }
    
    operator MotionPlain() const
    {
      return MotionPlain(v,MotionPlain::Vector3::Zero());
    }
    
    MotionTranslationTpl & operator=(const MotionTranslationTpl & other)
    {
      v = other.v;
      return *this;
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v_) const
    {
      v_.linear() += v;
    }
    
    // data
    Vector3 v;
    
  }; // struct MotionTranslationTpl
  
  template<typename S1, int O1, typename MotionDerived>
  inline typename MotionDerived::MotionPlain operator+(const MotionTranslationTpl<S1,O1> & m1, const MotionDense<MotionDerived> & m2)
  {
    return typename MotionDerived::MotionPlain(m2.linear() + m1.v, m2.angular());
  }
  
  template<typename Scalar, int Options> struct ConstraintTranslationTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintTranslationTpl<_Scalar,_Options> >
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
  }; // traits ConstraintTranslationTpl
  
  template<typename _Scalar, int _Options>
  struct ConstraintTranslationTpl : ConstraintBase< ConstraintTranslationTpl<_Scalar,_Options> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintTranslationTpl);
    
    enum { NV = 3, Options = traits<ConstraintTranslationTpl>::Options };
    typedef typename traits<ConstraintTranslationTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintTranslationTpl>::JointForce JointForce;
    typedef typename traits<ConstraintTranslationTpl>::DenseBase DenseBase;
    
    ConstraintTranslationTpl() {}
    
    template<typename S1, int O1>
    Motion operator*(const MotionTranslationTpl<S1,O1> & vj) const
    { return Motion(vj(), Motion::Vector3::Zero()); }
    
    template<typename Vector3Like>
    MotionTranslationTpl<Scalar,Options>
    operator*(const Eigen::MatrixBase<Vector3Like> & v) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
      return MotionTranslationTpl<Scalar,Options>(v);
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
      M.template middleRows<3>(LINEAR) = m.rotation ();
      M.template middleRows<3>(ANGULAR).setZero ();
      
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
    
  }; // struct ConstraintTranslationTpl
  
  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionTranslationTpl<S2,O2> & m2)
  {
    typedef typename MotionDerived::MotionPlain ReturnType;
    return ReturnType(m1.angular().cross(m2.v),
                      ReturnType::Vector3::Zero());
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
  
  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintTranslationTpl<S1,O1> >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintTranslationTpl<S1,O1>,MotionDerived >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  }
  
  
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
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTranslationTpl<Scalar,Options> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;

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
  struct JointDataTranslationTpl : public JointDataBase< JointDataTranslationTpl<_Scalar,_Options> >
  {
    typedef JointTranslationTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataTranslationTpl () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataTranslationTpl

  template<typename _Scalar, int _Options>
  struct JointModelTranslationTpl : public JointModelBase< JointModelTranslationTpl<_Scalar,_Options> >
  {
    typedef JointTranslationTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelTranslationTpl>::id;
    using JointModelBase<JointModelTranslationTpl>::idx_q;
    using JointModelBase<JointModelTranslationTpl>::idx_v;
    using JointModelBase<JointModelTranslationTpl>::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      data.M.translation(qs.template segment<NQ>(idx_q()));
    }
    
    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      calc(data,qs.derived());
      
      data.v() = vs.template segment<NQ>(idx_v());
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      data.U = I.template middleCols<3>(Inertia::LINEAR);
      data.Dinv = data.U.template middleRows<3>(Inertia::LINEAR).inverse();
      data.UDinv.template middleRows<3>(Inertia::LINEAR).setIdentity(); // can be put in data constructor
      data.UDinv.template middleRows<3>(Inertia::ANGULAR).noalias() = data.U.template middleRows<3>(Inertia::ANGULAR) * data.Dinv;
      
      if (update_I)
      {
        I.template block<3,3>(Inertia::ANGULAR,Inertia::ANGULAR)
        -= data.UDinv.template middleRows<3>(Inertia::ANGULAR) * I.template block<3,3>(Inertia::LINEAR, Inertia::ANGULAR);
        I.template middleCols<3>(Inertia::LINEAR).setZero();
        I.template block<3,3>(Inertia::LINEAR,Inertia::ANGULAR).setZero();
      }
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return sqrt(Eigen::NumTraits<Scalar>::epsilon());
    }
    
    static std::string classname() { return std::string("JointModelTranslation"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelTranslationTpl
  
} // namespace se3

#endif // ifndef __se3_joint_translation_hpp__
