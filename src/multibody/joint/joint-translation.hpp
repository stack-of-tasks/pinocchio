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
    
    operator Motion() const
    {
      return Motion (v,Motion::Vector3::Zero());
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
  
  struct ConstraintTranslationSubspace;
  template <>
  struct traits < ConstraintTranslationSubspace>
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<double,0> Quaternion_t;
    typedef SE3Tpl<double,0> SE3;
    typedef ForceTpl<double,0> Force;
    typedef MotionTpl<double,0> Motion;
    typedef Symmetric3Tpl<double,0> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef Eigen::Matrix<Scalar,3,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar,3,1,0> JointForce;
    typedef Eigen::Matrix<Scalar,6,3> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintTranslationSubspace
  
  struct ConstraintTranslationSubspace : ConstraintBase < ConstraintTranslationSubspace >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintTranslationSubspace);
    enum { NV = 3, Options = 0 };
    typedef traits<ConstraintTranslationSubspace>::JointMotion JointMotion;
    typedef traits<ConstraintTranslationSubspace>::JointForce JointForce;
    typedef traits<ConstraintTranslationSubspace>::DenseBase DenseBase;
    ConstraintTranslationSubspace() {}
    
    template<typename S1, int O1>
    Motion operator*(const MotionTranslationTpl<S1,O1> & vj) const
    { return Motion(vj(), Motion::Vector3::Zero()); }
    
    int nv_impl() const { return NV; }
    
    struct ConstraintTranspose
    {
      const ConstraintTranslationSubspace & ref;
      ConstraintTranspose(const ConstraintTranslationSubspace & ref) : ref(ref) {}
      
      template<typename Derived>
      typename ForceDense<Derived>::ConstLinearType
      operator* (const ForceDense<Derived> & phi)
      {
        return phi.linear();
      }
      
      
      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
      friend typename Eigen::Matrix <typename Eigen::MatrixBase<D>::Scalar, 3, -1>
      operator*( const ConstraintTranspose &, const Eigen::MatrixBase<D> & F )
      {
        assert(F.rows()==6);
        return  F.template middleRows <3> (Inertia::LINEAR);
      }
    }; // struct ConstraintTranspose
    
    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }
    
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.block <3,3> (Inertia::LINEAR, 0).setIdentity ();
      S.block <3,3> (Inertia::ANGULAR, 0).setZero ();
      return S;
    }
    
    Eigen::Matrix <double,6,3> se3Action (const SE3 & m) const
    {
      Eigen::Matrix <double,6,3> M;
      M.block <3,3> (Motion::LINEAR, 0) = m.rotation ();
      M.block <3,3> (Motion::ANGULAR, 0).setZero ();
      
      return M;
    }
    
    DenseBase motionAction(const Motion & m) const
    {
      const Motion::ConstAngularType w = m.angular();
      
      DenseBase res;
      skew(w,res.middleRows<3>(LINEAR));
      res.middleRows<3>(ANGULAR).setZero();
      
      return res;
    }
    
  }; // struct ConstraintTranslationSubspace
  
  template<typename D>
  Motion operator* (const ConstraintTranslationSubspace &, const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    return Motion (v, Motion::Vector3::Zero ());
  }
  
  
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
  inline Eigen::Matrix <double, 6, 3> operator* (const Inertia & Y, const ConstraintTranslationSubspace &)
  {
    Eigen::Matrix <double, 6, 3> M;
    M.block <3,3> (Inertia::ANGULAR, 0) = alphaSkew(Y.mass (), Y.lever ());
    //    M.block <3,3> (Inertia::LINEAR, 0) = Y.mass () * Inertia::Matrix3::Identity ();
    M.block <3,3> (Inertia::LINEAR, 0).setZero ();
    M.block <3,3> (Inertia::LINEAR, 0).diagonal ().fill (Y.mass ());
    
    return M;
  }
  
  namespace internal
  {
    template<>
    struct SE3GroupAction<ConstraintTranslationSubspace >
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
    
    template<typename MotionDerived>
    struct MotionAlgebraAction<ConstraintTranslationSubspace,MotionDerived>
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
  }
  
  
  struct JointTranslation;
  template<>
  struct traits<JointTranslation>
  {
    enum {
      NQ = 3,
      NV = 3
    };
    typedef double Scalar;
    enum { Options = 0 };
    typedef JointDataTranslation JointDataDerived;
    typedef JointModelTranslation JointModelDerived;
    typedef ConstraintTranslationSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionTranslationTpl<Scalar,Options> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  }; // traits JointTranslation
  
  template<> struct traits<JointDataTranslation> { typedef JointTranslation JointDerived; };
  template<> struct traits<JointModelTranslation> { typedef JointTranslation JointDerived; };

  struct JointDataTranslation : public JointDataBase<JointDataTranslation>
  {
    typedef JointTranslation JointDerived;
    SE3_JOINT_TYPEDEF;

    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Matrix<double,3,3> Matrix3;
    typedef Eigen::Matrix<double,3,1> Vector3;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataTranslation () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataTranslation

  struct JointModelTranslation : public JointModelBase<JointModelTranslation>
  {
    typedef JointTranslation JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelTranslation>::id;
    using JointModelBase<JointModelTranslation>::idx_q;
    using JointModelBase<JointModelTranslation>::idx_v;
    using JointModelBase<JointModelTranslation>::setIndexes;
    typedef Motion::Vector3 Vector3;

    JointDataDerived createData() const { return JointDataDerived(); }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs) const
    {
      data.M.translation (qs.segment<NQ>(idx_q ()));
    }
    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      data.M.translation (qs.segment<NQ> (idx_q ()));
      data.v () = vs.segment<NQ> (idx_v ());
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.block<6,3> (0,Inertia::LINEAR);
      data.Dinv = I.block<3,3> (Inertia::LINEAR,Inertia::LINEAR).inverse();
      data.UDinv.middleRows<3> (Inertia::LINEAR).setIdentity(); // can be put in data constructor
      data.UDinv.middleRows<3> (Inertia::ANGULAR) = data.U.block<3,3> (Inertia::ANGULAR, 0) * data.Dinv;
      
      if (update_I)
      {
        I.block<6,3> (0,Inertia::LINEAR).setZero();
        I.block<3,3> (Inertia::LINEAR,Inertia::ANGULAR).setZero();
        I.block<3,3> (Inertia::ANGULAR,Inertia::ANGULAR) -= data.UDinv.middleRows<3> (Inertia::ANGULAR) * I.block<3,3> (Inertia::LINEAR, Inertia::ANGULAR);
      }
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return sqrt(Eigen::NumTraits<Scalar>::epsilon());
    }
    
    static std::string classname() { return std::string("JointModelTranslation"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelTranslation
  
} // namespace se3

#endif // ifndef __se3_joint_translation_hpp__
