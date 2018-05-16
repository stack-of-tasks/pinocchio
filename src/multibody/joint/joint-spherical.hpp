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

    operator Motion() const
    {
      return Motion(Motion::Vector3::Zero(), w);
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

  template<typename Scalar, int Options> struct ConstraintRotationalSubspace;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintRotationalSubspace<_Scalar,_Options> >
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
  struct ConstraintRotationalSubspace : public ConstraintBase< ConstraintRotationalSubspace<_Scalar,_Options> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRotationalSubspace);
    enum { NV = 3 };
    typedef typename traits<ConstraintRotationalSubspace>::JointMotion JointMotion;
    typedef typename traits<ConstraintRotationalSubspace>::JointForce JointForce;
    typedef typename traits<ConstraintRotationalSubspace>::DenseBase DenseBase;
    
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
      operator*(const Eigen::MatrixBase<MatrixDerived> & F)
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

  }; // struct ConstraintRotationalSubspace

              
  template<typename Scalar, int Options, typename Vector3Like>
  MotionSpherical<Scalar,Options>
  operator*(const ConstraintRotationalSubspace<Scalar,Options> &,
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
            const ConstraintRotationalSubspace<S2,O2> &)
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
  inline const typename SizeDepType<3>::ColsReturn<M6Like>::ConstType
  operator*(const Eigen::MatrixBase<M6Like> & Y,
            const ConstraintRotationalSubspace<S2,O2> &)
  {
    return Y.derived().template middleCols<3>(Inertia::ANGULAR);
  }
  
  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintRotationalSubspace<S1,O1> >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintRotationalSubspace<S1,O1>,MotionDerived >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  }

  struct JointSpherical;
  template<>
  struct traits<JointSpherical>
  {
    enum {
      NQ = 4,
      NV = 3
    };
    typedef double Scalar;
    enum { Options = 0 };
    typedef JointDataSpherical JointDataDerived;
    typedef JointModelSpherical JointModelDerived;
    typedef ConstraintRotationalSubspace<Scalar,Options> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionSpherical<Scalar,Options> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };
  template<> struct traits<JointDataSpherical> { typedef JointSpherical JointDerived; };
  template<> struct traits<JointModelSpherical> { typedef JointSpherical JointDerived; };

  struct JointDataSpherical : public JointDataBase<JointDataSpherical>
  {
    typedef JointSpherical JointDerived;
    SE3_JOINT_TYPEDEF;

    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Matrix<double,3,1> Vector3;
    typedef Eigen::Quaternion<double> Quaternion;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataSpherical () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataSpherical

  struct JointModelSpherical : public JointModelBase<JointModelSpherical>
  {
    typedef JointSpherical JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelSpherical>::id;
    using JointModelBase<JointModelSpherical>::idx_q;
    using JointModelBase<JointModelSpherical>::idx_v;
    using JointModelBase<JointModelSpherical>::setIndexes;
    typedef Motion::Vector3 Vector3;
    typedef Eigen::Map<const SE3::Quaternion_t> ConstQuaternionMap_t;

    JointDataDerived createData() const { return JointDataDerived(); }

    template<typename V>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<V> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,V);
      //using std::sqrt;
      typedef Eigen::Map<const SE3::Quaternion_t> ConstQuaternionMap_t;

      ConstQuaternionMap_t quat(q_joint.derived().data());
      //assert(std::fabs(quat.coeffs().squaredNorm()-1.) <= sqrt(Eigen::NumTraits<typename V::Scalar>::epsilon())); TODO: check validity of the rhs precision
      assert(std::fabs(quat.coeffs().squaredNorm()-1.) <= 1e-4);
      
      M.rotation(quat.matrix());
      M.translation().setZero();
    }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs) const
    {
      typedef Eigen::Map<const SE3::Quaternion_t> ConstQuaternionMap_t;
      
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());
      
      ConstQuaternionMap_t quat(q.data());
      data.M.rotation (quat.matrix());
    }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      typedef Eigen::Map<const SE3::Quaternion_t> ConstQuaternionMap_t;
      
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      data.v () = vs.segment<NV> (idx_v ());

      ConstQuaternionMap_t quat(q.data());
      data.M.rotation (quat.matrix ());
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.block<6,3> (0,Inertia::ANGULAR);
      data.Dinv = I.block<3,3> (Inertia::ANGULAR,Inertia::ANGULAR).inverse();
      data.UDinv.middleRows<3> (Inertia::ANGULAR).setIdentity(); // can be put in data constructor
      data.UDinv.middleRows<3> (Inertia::LINEAR) = data.U.block<3,3> (Inertia::LINEAR, 0) * data.Dinv;
      
      if (update_I)
      {
        I.block<6,3> (0,Inertia::ANGULAR).setZero();
        I.block<3,3> (Inertia::ANGULAR,Inertia::LINEAR).setZero();
        I.block<3,3> (Inertia::LINEAR,Inertia::LINEAR) -= data.UDinv.middleRows<3> (Inertia::LINEAR) * I.block<3,3> (Inertia::ANGULAR, Inertia::LINEAR);
      }
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelSpherical"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelSpherical

} // namespace se3

#endif // ifndef __se3_joint_spherical_hpp__
