//
// Copyright (c) 2015-2016 CNRS
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

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/quaternion.hpp"

namespace se3
{

  struct MotionSpherical;
  
  template <>
  struct traits< MotionSpherical >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef MotionTpl<double,0> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits MotionSpherical

  struct MotionSpherical : MotionBase < MotionSpherical >
  {
    MOTION_TYPEDEF(MotionSpherical);

    MotionSpherical ()                   : w (Motion::Vector3(NAN, NAN, NAN)) {}
    MotionSpherical (const Motion::Vector3 & w) : w (w)  {}
    Motion::Vector3 w;

    Motion::Vector3 & operator() () { return w; }
    const Motion::Vector3 & operator() () const { return w; }

    operator Motion() const
    {
      return Motion (Motion::Vector3::Zero (), w);
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v) const
    {
      v.angular() += w;
    }
  }; // struct MotionSpherical

  inline const MotionSpherical operator+ (const MotionSpherical & m, const BiasZero & )
  { return m; }

  inline Motion operator+ (const MotionSpherical & m1, const Motion & m2)
  {
    return Motion( m2.linear(), m2.angular() + m1.w);
  }

  struct ConstraintRotationalSubspace;
  template <>
  struct traits < struct ConstraintRotationalSubspace >
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
  }; // struct traits struct ConstraintRotationalSubspace

  struct ConstraintRotationalSubspace : public ConstraintBase< ConstraintRotationalSubspace >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintRotationalSubspace);
    enum { NV = 3, Options = 0 };
    typedef traits<ConstraintRotationalSubspace>::JointMotion JointMotion;
    typedef traits<ConstraintRotationalSubspace>::JointForce JointForce;
    typedef traits<ConstraintRotationalSubspace>::DenseBase DenseBase;

    /// Missing operator*
    // Motion operator* (const MotionSpherical & vj) const
    // { return ??; }

    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      
      template<typename Derived>
      typename ForceDense<Derived>::ConstAngularType
      operator* (const ForceDense<Derived> & phi)
      {  return phi.angular();  }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
      friend typename Eigen::Matrix <typename Eigen::MatrixBase<D>::Scalar, 3, -1>
      operator*( const TransposeConst &, const Eigen::MatrixBase<D> & F )
      {
        assert(F.rows()==6);
        return F.template middleRows <3> (Inertia::ANGULAR);
      }
    };

    TransposeConst transpose() const { return TransposeConst(); }

    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.block <3,3> (Inertia::LINEAR, 0).setZero ();
      S.block <3,3> (Inertia::ANGULAR, 0).setIdentity ();
      return S;
    }

    Eigen::Matrix <double,6,3> se3Action (const SE3 & m) const
    {
      Eigen::Matrix <double,6,3> X_subspace;
      X_subspace.block <3,3> (Motion::LINEAR, 0) = cross(m.translation(),m.rotation ());
      X_subspace.block <3,3> (Motion::ANGULAR, 0) = m.rotation ();

      return X_subspace;
    }
    
    DenseBase motionAction(const Motion & m) const
    {
      const Motion::ConstLinearType v = m.linear();
      const Motion::ConstAngularType w = m.angular();
      
      DenseBase res;
      skew(v,res.middleRows<3>(LINEAR));
      skew(w,res.middleRows<3>(ANGULAR));
      
      return res;
    }

  }; // struct ConstraintRotationalSubspace

  template<typename D>
  Motion operator* (const ConstraintRotationalSubspace&, const Eigen::MatrixBase<D>& v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    return Motion (Motion::Vector3::Zero (), v);
  }


  inline Motion operator^ (const Motion & m1, const MotionSpherical & m2)
  {
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Eigen::Matrix <double, 6, 3> operator* (const Inertia & Y, const ConstraintRotationalSubspace & )
  {
    Eigen::Matrix <double, 6, 3> M;
    //    M.block <3,3> (Inertia::LINEAR, 0) = - Y.mass () * skew(Y.lever ());
    M.block <3,3> (Inertia::LINEAR, 0) = alphaSkew ( -Y.mass (),  Y.lever ());
    M.block <3,3> (Inertia::ANGULAR, 0) = (Y.inertia () - Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ())).matrix();
    return M;
  }

  /* [ABA] Y*S operator*/
  inline SizeDepType<3>::ColsReturn<Inertia::Matrix6>::ConstType operator* (const Inertia::Matrix6 & Y, const ConstraintRotationalSubspace &)
  {
    return Y.middleCols<3>(Inertia::ANGULAR);
  }
  
  namespace internal
  {
    template<>
    struct SE3GroupAction<ConstraintRotationalSubspace >
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
    
    template<>
    struct MotionAlgebraAction<ConstraintRotationalSubspace>
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
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
    typedef JointDataSpherical JointDataDerived;
    typedef JointModelSpherical JointModelDerived;
    typedef ConstraintRotationalSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionSpherical Motion_t;
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
