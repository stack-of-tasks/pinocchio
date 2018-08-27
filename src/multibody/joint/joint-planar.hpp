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

#ifndef __se3_joint_planar_hpp__
#define __se3_joint_planar_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace se3
{
  
  template<typename Scalar, int Options = 0> struct MotionPlanarTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< MotionPlanarTpl<_Scalar,_Options> >
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
  }; // traits MotionPlanarTpl

  template<typename _Scalar, int _Options>
  struct MotionPlanarTpl : MotionBase< MotionPlanarTpl<_Scalar,_Options> >
  {
    MOTION_TYPEDEF_TPL(MotionPlanarTpl);

    MotionPlanarTpl () : m_x_dot(NAN), m_y_dot(NAN), m_theta_dot(NAN) {}
    
    MotionPlanarTpl (Scalar x_dot, Scalar y_dot, Scalar theta_dot)
    : m_x_dot(x_dot), m_y_dot(y_dot), m_theta_dot(theta_dot)
    {}

    operator MotionPlain() const
    {
      return MotionPlain(typename MotionPlain::Vector3(m_x_dot,m_y_dot,Scalar(0)),
                         typename MotionPlain::Vector3(Scalar(0),Scalar(0),m_theta_dot));
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v) const
    {
      v.linear()[0] += m_x_dot;
      v.linear()[1] += m_y_dot;
      v.angular()[2] += m_theta_dot;
    }

    // data
    Scalar m_x_dot;
    Scalar m_y_dot;
    Scalar m_theta_dot;
    
  }; // struct MotionPlanarTpl
  
  template<typename Scalar, int Options, typename MotionDerived>
  inline typename MotionDerived::MotionPlain
  operator+(const MotionPlanarTpl<Scalar,Options> & m1, const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain result(m2);
    result.linear()[0] += m1.m_x_dot;
    result.linear()[1] += m1.m_y_dot;

    result.angular()[2] += m1.m_theta_dot;

    return result;
  }

  template<typename Scalar, int Options> struct ConstraintPlanarTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< ConstraintPlanarTpl<_Scalar,_Options> >
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
  }; // struct traits ConstraintPlanarTpl

  template<typename _Scalar, int _Options>
  struct ConstraintPlanarTpl : ConstraintBase< ConstraintPlanarTpl<_Scalar,_Options> >
  {

    SPATIAL_TYPEDEF_TEMPLATE(ConstraintPlanarTpl);
    enum { NV = 3, Options = 0 }; // to check
    typedef typename traits<ConstraintPlanarTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintPlanarTpl>::JointForce JointForce;
    typedef typename traits<ConstraintPlanarTpl>::DenseBase DenseBase;

    template<typename S1, int O1>
    typename MotionPlanarTpl<S1,O1>::MotionPlain
    operator*(const MotionPlanarTpl<S1,O1> & vj) const
    { return vj; }

    int nv_impl() const { return NV; }

    struct ConstraintTranspose
    {
      const ConstraintPlanarTpl & ref;
      ConstraintTranspose(const ConstraintPlanarTpl & ref) : ref(ref) {}

      template<typename Derived>
      typename ForceDense<Derived>::Vector3 operator* (const ForceDense<Derived> & phi)
      {
        typedef typename ForceDense<Derived>::Vector3 Vector3;
        return Vector3(phi.linear()[0], phi.linear()[1], phi.angular()[2]);
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename Derived>
      friend typename Eigen::Matrix <typename Eigen::MatrixBase<Derived>::Scalar, 3, Derived::ColsAtCompileTime>
      operator*(const ConstraintTranspose &, const Eigen::MatrixBase<Derived> & F)
      {
        typedef typename Eigen::MatrixBase<Derived>::Scalar Scalar;
        typedef Eigen::Matrix<Scalar, 3, Derived::ColsAtCompileTime> MatrixReturnType;
        assert(F.rows()==6);

        MatrixReturnType result(3, F.cols ());
        result.template topRows<2>() = F.template topRows<2>();
        result.template bottomRows<1>() = F.template bottomRows<1>();
        return result;
      }
      
    }; // struct ConstraintTranspose

    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.template block<3,3>(Inertia::LINEAR, 0).setZero ();
      S.template block<3,3>(Inertia::ANGULAR, 0).setZero ();
      S(Inertia::LINEAR + 0,0) = Scalar(1);
      S(Inertia::LINEAR + 1,1) = Scalar(1);
      S(Inertia::ANGULAR + 2,2) = Scalar(1);
      return S;
    }

    template<typename S1, int O1>
    DenseBase se3Action(const SE3Tpl<S1,O1> & m) const
    {
      DenseBase X_subspace;
      X_subspace.template block <3,2>(Motion::LINEAR, 0) = m.rotation ().template leftCols <2> ();
      X_subspace.template block <3,1>(Motion::LINEAR, 2) = m.translation().cross(m.rotation ().template rightCols<1>());

      X_subspace.template block <3,2>(Motion::ANGULAR, 0).setZero ();
      X_subspace.template block <3,1>(Motion::ANGULAR, 2) = m.rotation ().template rightCols<1>();

      return X_subspace;
    }

    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      const typename MotionDerived::ConstLinearType v = m.linear();
      const typename MotionDerived::ConstAngularType w = m.angular();
      DenseBase res(DenseBase::Zero());
      
      res(0,1) = -w[2]; res(0,2) = v[1];
      res(1,0) = w[2]; res(1,2) = -v[0];
      res(2,0) = -w[1]; res(2,1) = w[0];
      res(3,2) = w[1];
      res(4,2) = -w[0];
      
      return res;
    }
  }; // struct ConstraintPlanarTpl

  template<typename Scalar, int Options, typename MatrixDerived>
  MotionTpl<Scalar,Options> operator* (const ConstraintPlanarTpl<Scalar,Options> &, const Eigen::MatrixBase<MatrixDerived> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(MatrixDerived,3);
    
    typedef MotionTpl<Scalar,Options> ReturnType;
    ReturnType result(ReturnType::Zero());
    result.linear().template head<2> () = v.template topRows<2>();
    result.angular().template tail<1> () = v.template bottomRows<1>();
    return result;
  }

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain operator^ (const MotionDense<MotionDerived> & m1, const MotionPlanarTpl<S2,O2> & m2)
  {
    typename MotionDerived::MotionPlain result;
    typedef typename MotionDerived::Scalar Scalar;

    const typename MotionDerived::ConstLinearType & m1_t = m1.linear();
    const typename MotionDerived::ConstAngularType & m1_w = m1.angular();

    result.angular()
    << m1_w(1) * m2.m_theta_dot
    , - m1_w(0) * m2.m_theta_dot
    , Scalar(0);
    
    result.linear()
    << m1_t(1) * m2.m_theta_dot - m1_w(2) * m2.m_y_dot
    , - m1_t(0) * m2.m_theta_dot + m1_w(2) * m2.m_x_dot
    , m1_w(0) * m2.m_y_dot - m1_w(1) * m2.m_x_dot;

    return result;
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline typename Eigen::Matrix<S1,6,3,O1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPlanarTpl<S2,O2> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    typedef typename Inertia::Scalar Scalar;
    typedef Eigen::Matrix<S1,6,3,O1> ReturnType;
    
    ReturnType M;
    const Scalar mass = Y.mass();
    const typename Inertia::Vector3 & com = Y.lever();
    const typename Inertia::Symmetric3 & inertia = Y.inertia();

    M.template topLeftCorner<3,3>().setZero();
    M.template topLeftCorner<2,2>().diagonal().fill(mass);

    const typename Inertia::Vector3 mc(mass * com);
    M.template rightCols<1>().template head<2>() << -mc(1), mc(0);

    M.template bottomLeftCorner<3,2>() << 0., -mc(2), mc(2), 0., -mc(1), mc(0);
    M.template rightCols<1>().template tail<3>() = inertia.data().template tail<3>();
    M.template rightCols<1>()[3] -= mc(0)*com(2);
    M.template rightCols<1>()[4] -= mc(1)*com(2);
    M.template rightCols<1>()[5] += mass*(com(0)*com(0) + com(1)*com(1));

    return M;
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  //  inline Eigen::Matrix<double,6,1>
  
  template<typename M6Like, typename S2, int O2>
  inline Eigen::Matrix<S2,6,3,O2>
  operator*(const Eigen::MatrixBase<M6Like> & Y,
            const ConstraintPlanarTpl<S2,O2> &)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
    typedef Eigen::Matrix<S2,6,3,O2> Matrix63;
    
    Matrix63 IS;
    IS.template leftCols<2>() = Y.template leftCols<2>();
    IS.template rightCols<1>() = Y.template rightCols<1>();
    
    return IS;
  }
  
  namespace internal
  {
    template<typename S1, int O1>
    struct SE3GroupAction< ConstraintPlanarTpl<S1,O1> >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
    
    template<typename S1, int O1, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintPlanarTpl<S1,O1>,MotionDerived >
    { typedef Eigen::Matrix<S1,6,3,O1> ReturnType; };
  }

  template<typename Scalar, int Options> struct JointPlanarTpl;
  
  template<typename _Scalar, int _Options>
  struct traits< JointPlanarTpl<_Scalar,_Options> >
  {
    enum {
      NQ = 4,
      NV = 3
    };
    enum { Options = _Options };
    typedef _Scalar Scalar;
    typedef JointDataPlanarTpl<Scalar,Options> JointDataDerived;
    typedef JointModelPlanarTpl<Scalar,Options> JointModelDerived;
    typedef ConstraintPlanarTpl<Scalar,Options> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionPlanarTpl<Scalar,Options> Motion_t;
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
  struct traits< JointDataPlanarTpl<Scalar,Options> > { typedef JointPlanarTpl<Scalar,Options> JointDerived; };
  template<typename Scalar, int Options>
  struct traits< JointModelPlanarTpl<Scalar,Options> > { typedef JointPlanarTpl<Scalar,Options> JointDerived; };

  template<typename _Scalar, int _Options>
  struct JointDataPlanarTpl : public JointDataBase< JointDataPlanarTpl<_Scalar,_Options> >
  {
    typedef JointPlanarTpl<_Scalar,_Options> JointDerived;
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

    JointDataPlanarTpl () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataPlanarTpl

  template<typename _Scalar, int _Options>
  struct JointModelPlanarTpl : public JointModelBase< JointModelPlanarTpl<_Scalar,_Options> >
  {
    typedef JointPlanarTpl<_Scalar,_Options> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelPlanarTpl>::id;
    using JointModelBase<JointModelPlanarTpl>::idx_q;
    using JointModelBase<JointModelPlanarTpl>::idx_v;
    using JointModelBase<JointModelPlanarTpl>::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename V>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<V> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,V);
      
      const double& c_theta = q_joint(2),
                    s_theta = q_joint(3);
      
      M.rotation().template topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      M.translation().template head<2>() = q_joint.template head<2>();
    }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(ConfigVector);
      
      typedef typename ConfigVector::Scalar Scalar;
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type & q = qs.template segment<NQ>(idx_q());

      const Scalar
      &c_theta = q(2),
      &s_theta = q(3);

      data.M.rotation().template topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation().template head<2>() = q.template head<2>();

    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(TangentVector);
      calc(data,qs.derived());
      
      typename TangentVector::template ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.template segment<NV>(idx_v ());

      data.v.m_x_dot = q_dot(0);
      data.v.m_y_dot = q_dot(1);
      data.v.m_theta_dot = q_dot(2);
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      data.U.template leftCols<2>() = I.template leftCols<2>();
      data.U.template rightCols<1>() = I.template rightCols<1>();
      Eigen::Matrix<S2,3,3,O2> tmp;
      tmp.template leftCols<2>() = data.U.template topRows<2>().transpose();
      tmp.template rightCols<1>() = data.U.template bottomRows<1>();
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

    static std::string classname() { return std::string("JointModelPlanar");}
    std::string shortname() const { return classname(); }

  }; // struct JointModelPlanarTpl

} // namespace se3

#endif // ifndef __se3_joint_planar_hpp__
