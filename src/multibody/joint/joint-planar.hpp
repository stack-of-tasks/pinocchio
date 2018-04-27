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

#ifndef __se3_joint_planar_hpp__
#define __se3_joint_planar_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"

#include <stdexcept>

namespace se3
{

  struct MotionPlanar;
  template <>
  struct traits< MotionPlanar >
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
  }; // traits MotionPlanar

  struct MotionPlanar : MotionBase < MotionPlanar >
  {
    MOTION_TYPEDEF(MotionPlanar);

    MotionPlanar () : x_dot_(NAN), y_dot_(NAN), theta_dot_(NAN)      {}
    MotionPlanar (Scalar x_dot, Scalar y_dot, Scalar theta_dot) : x_dot_(x_dot), y_dot_(y_dot), theta_dot_(theta_dot)  {}
    Scalar x_dot_;
    Scalar y_dot_;
    Scalar theta_dot_;

    operator Motion() const
    {
      return Motion (Motion::Vector3(x_dot_, y_dot_, 0.), Motion::Vector3(0., 0., theta_dot_));
    }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v) const
    {
      v.linear()[0] += x_dot_;
      v.linear()[1] += y_dot_;
      v.angular()[2] += theta_dot_;
    }

  }; // struct MotionPlanar

  inline const MotionPlanar operator+ (const MotionPlanar & m, const BiasZero &)
  { return m; }

  
  inline Motion operator+ (const MotionPlanar & m1, const Motion & m2)
  {
    Motion result (m2);
    result.linear ()[0] += m1.x_dot_;
    result.linear ()[1] += m1.y_dot_;

    result.angular ()[2] += m1.theta_dot_;

    return result;
  }


  struct ConstraintPlanar;
  template <>
  struct traits < ConstraintPlanar >
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
  }; // struct traits ConstraintPlanar


  struct ConstraintPlanar : ConstraintBase < ConstraintPlanar >
  {

    SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintPlanar);
    enum { NV = 3, Options = 0 }; // to check
    typedef traits<ConstraintPlanar>::JointMotion JointMotion;
    typedef traits<ConstraintPlanar>::JointForce JointForce;
    typedef traits<ConstraintPlanar>::DenseBase DenseBase;


    Motion operator* (const MotionPlanar & vj) const
    { return vj; }

    int nv_impl() const { return NV; }

    struct ConstraintTranspose
    {
      const ConstraintPlanar & ref;
      ConstraintTranspose(const ConstraintPlanar & ref) : ref(ref) {}

      template<typename Derived>
      typename ForceDense<Derived>::Vector3 operator* (const ForceDense<Derived> & phi)
      {
        return Force::Vector3 (phi.linear()[0], phi.linear()[1], phi.angular()[2]);
      }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
      friend typename Eigen::Matrix <typename Eigen::MatrixBase<D>::Scalar, 3, -1>
      operator*( const ConstraintTranspose &, const Eigen::MatrixBase<D> & F )
      {
        typedef Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar, 3, -1> MatrixReturnType;
        assert(F.rows()==6);

        MatrixReturnType result (3, F.cols ());
        result.template topRows <2> () = F.template topRows <2> ();
        result.template bottomRows <1> () = F.template bottomRows <1> ();
        return result;
      }
      
    }; // struct ConstraintTranspose

    ConstraintTranspose transpose () const { return ConstraintTranspose(*this); }

    DenseBase matrix_impl() const
    {
      DenseBase S;
      S.block <3,3> (Inertia::LINEAR, 0).setZero ();
      S.block <3,3> (Inertia::ANGULAR, 0).setZero ();
      S(Inertia::LINEAR + 0,0) = 1.;
      S(Inertia::LINEAR + 1,1) = 1.;
      S(Inertia::ANGULAR + 2,2) = 1.;
      return S;
    }

    Eigen::Matrix <Scalar,6,3> se3Action (const SE3 & m) const
    {
      Eigen::Matrix <double,6,3> X_subspace;
      X_subspace.block <3,2> (Motion::LINEAR, 0) = m.rotation ().leftCols <2> ();
      X_subspace.block <3,1> (Motion::LINEAR, 2) = skew (m.translation ()) * m.rotation ().rightCols <1> ();

      X_subspace.block <3,2> (Motion::ANGULAR, 0).setZero ();
      X_subspace.block <3,1> (Motion::ANGULAR, 2) = m.rotation ().rightCols <1> ();

      return X_subspace;
    }

    DenseBase motionAction(const Motion & m) const
    {
      const Motion::ConstLinearType v = m.linear();
      const Motion::ConstAngularType w = m.angular();
      DenseBase res(DenseBase::Zero());
      
      res(0,1) = -w[2]; res(0,2) = v[1];
      res(1,0) = w[2]; res(1,2) = -v[0];
      res(2,0) = -w[1]; res(2,1) = w[0];
      res(3,2) = w[1];
      res(4,2) = -w[0];
      
      return res;
    }
  }; // struct ConstraintPlanar

  template<typename D>
  Motion operator* (const ConstraintPlanar &, const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    Motion result (Motion::Zero ());
    result.linear ().template head<2> () = v.template topRows<2> ();
    result.angular ().template tail<1> () = v.template bottomRows<1> ();
    return result;
  }


  inline Motion operator^ (const Motion & m1, const MotionPlanar & m2)
  {
    Motion result;

    const Motion::Vector3 & m1_t = m1.linear();
    const Motion::Vector3 & m1_w = m1.angular();

    result.angular () << m1_w(1) * m2.theta_dot_, - m1_w(0) * m2.theta_dot_, 0.;
    result.linear () << m1_t(1) * m2.theta_dot_ - m1_w(2) * m2.y_dot_, - m1_t(0) * m2.theta_dot_ + m1_w(2) * m2.x_dot_, m1_w(0) * m2.y_dot_ - m1_w(1) * m2.x_dot_;

    return result;
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Eigen::Matrix <Inertia::Scalar, 6, 3> operator* (const Inertia & Y, const ConstraintPlanar &)
  {
    Eigen::Matrix <Inertia::Scalar, 6, 3> M;
    const double mass = Y.mass ();
    const Inertia::Vector3 & com = Y.lever ();
    const Symmetric3 & inertia = Y.inertia ();

    M.topLeftCorner <3,3> ().setZero ();
    M.topLeftCorner <2,2> ().diagonal ().fill (mass);

    Inertia::Vector3 mc (mass * com);
    M.rightCols <1> ().head <2> () << -mc(1), mc(0);

    M.bottomLeftCorner <3,2> () << 0., -mc(2), mc(2), 0., -mc(1), mc(0);
    M.rightCols <1> ().tail <3> () = inertia.data ().tail <3> ();
    M.rightCols <1> ()[3] -= mc(0)*com(2);
    M.rightCols <1> ()[4] -= mc(1)*com(2);
    M.rightCols <1> ()[5] += mass*(com(0)*com(0) + com(1)*com(1));

    return M;
  }
  
  /* [ABA] Y*S operator (Inertia Y,Constraint S) */
  //  inline Eigen::Matrix<double,6,1>
  inline
  Eigen::Matrix<Inertia::Scalar, 6, 3>
  operator* (const Inertia::Matrix6 & Y, const ConstraintPlanar &)
  {
    typedef Eigen::Matrix<Inertia::Scalar, 6, 3> Matrix63;
    Matrix63 IS;
    
    IS.leftCols <2> () = Y.leftCols <2> ();
    IS.rightCols <1> () = Y.rightCols <1> ();
    
    return IS;
  }
  
  

  namespace internal
  {
    template<>
    struct SE3GroupAction<ConstraintPlanar >
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
    
    template<>
    struct MotionAlgebraAction<ConstraintPlanar>
    { typedef Eigen::Matrix<double,6,3> ReturnType; };
  }

  struct JointPlanar;
  template<>
  struct traits<JointPlanar>
  {
    enum {
      NQ = 4,
      NV = 3
    };
    typedef double Scalar;
    typedef JointDataPlanar JointDataDerived;
    typedef JointModelPlanar JointModelDerived;
    typedef ConstraintPlanar Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionPlanar Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };
  template<> struct traits<JointDataPlanar> { typedef JointPlanar JointDerived; };
  template<> struct traits<JointModelPlanar> { typedef JointPlanar JointDerived; };

  struct JointDataPlanar : public JointDataBase<JointDataPlanar>
  {
    typedef JointPlanar JointDerived;
    SE3_JOINT_TYPEDEF;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t
    
    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataPlanar () : M(1), U(), Dinv(), UDinv() {}

  }; // struct JointDataPlanar

  struct JointModelPlanar : public JointModelBase<JointModelPlanar>
  {
    typedef JointPlanar JointDerived;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelPlanar>::id;
    using JointModelBase<JointModelPlanar>::idx_q;
    using JointModelBase<JointModelPlanar>::idx_v;
    using JointModelBase<JointModelPlanar>::setIndexes;

    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename V>
    inline void forwardKinematics(Transformation_t & M, const Eigen::MatrixBase<V> & q_joint) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,V);
      
      const double& c_theta = q_joint(2),
                    s_theta = q_joint(3);
      
      M.rotation().topLeftCorner<2,2>() << c_theta, -s_theta, s_theta, c_theta;
      M.translation().head<2>() = q_joint.template head<2>();
    }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());

      const double& c_theta = q(2),
                    s_theta = q(3);

      data.M.rotation ().topLeftCorner <2,2> () << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation ().head <2> () = q.head<2> ();

    }

    void calc (JointDataDerived & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      const double& c_theta = q(2),
                    s_theta = q(3);

      data.M.rotation ().topLeftCorner <2,2> () << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation ().head <2> () = q.head<2> ();

      data.v.x_dot_ = q_dot(0);
      data.v.y_dot_ = q_dot(1);
      data.v.theta_dot_ = q_dot(2);
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U.leftCols<2> () = I.leftCols<2> ();
      data.U.rightCols<1> () = I.rightCols<1> ();
      Inertia::Matrix3 tmp;
      tmp.leftCols<2> () = data.U.topRows<2> ().transpose();
      tmp.rightCols<1> () = data.U.bottomRows<1> ();
      data.Dinv = tmp.inverse();
      data.UDinv.noalias() = data.U * data.Dinv;
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    ConfigVector_t::Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      typedef ConfigVector_t::Scalar Scalar;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname() { return std::string("JointModelPlanar");}
    std::string shortname() const { return classname(); }

  }; // struct JointModelPlanar

} // namespace se3

#endif // ifndef __se3_joint_planar_hpp__
