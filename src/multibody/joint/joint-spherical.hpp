//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-dense.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/math/quaternion.hpp"

namespace se3
{

  struct JointDataSpherical;
  struct JointModelSpherical;

  struct MotionSpherical;
  template <>
  struct traits< MotionSpherical >
  {
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
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
  }; // traits MotionSpherical

  struct MotionSpherical : MotionBase < MotionSpherical >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(MotionSpherical);

    MotionSpherical ()                   : w (Motion::Vector3(NAN, NAN, NAN)) {}
    MotionSpherical (const Motion::Vector3 & w) : w (w)  {}
    Motion::Vector3 w;

    Motion::Vector3 & operator() () { return w; }
    const Motion::Vector3 & operator() () const { return w; }

    operator Motion() const
    {
      return Motion (Motion::Vector3::Zero (), w);
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
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
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
    typedef Eigen::Matrix<Scalar_t,3,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar_t,3,1,0> JointForce;
    typedef Eigen::Matrix<Scalar_t,6,3> DenseBase;
  }; // struct traits struct ConstraintRotationalSubspace

  struct ConstraintRotationalSubspace
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
      Force::Vector3 operator* (const Force & phi)
      {  return phi.angular ();  }

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

    operator ConstraintXd () const
    {
      Eigen::Matrix<double,6,3> S;
      S.block <3,3> (Inertia::LINEAR, 0).setZero ();
      S.block <3,3> (Inertia::ANGULAR, 0).setIdentity ();
      return ConstraintXd(S);
    }

    Eigen::Matrix <double,6,3> se3Action (const SE3 & m) const
    {
      Eigen::Matrix <double,6,3> X_subspace;
      X_subspace.block <3,3> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ();
      X_subspace.block <3,3> (Motion::ANGULAR, 0) = m.rotation ();

      return X_subspace;
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
    M.block <3,3> (Inertia::ANGULAR, 0) = (Inertia::Matrix3)(Y.inertia () - Symmetric3::AlphaSkewSquare(Y.mass (), Y.lever ()));
    return M;
  }

  namespace internal
  {
    template<>
    struct ActionReturn<ConstraintRotationalSubspace >
    { typedef Eigen::Matrix<double,6,3> Type; };
  }

  struct JointSpherical;
  template<>
  struct traits<JointSpherical>
  {
    enum {
      NQ = 4,
      NV = 3
    };
    typedef JointDataSpherical JointData;
    typedef JointModelSpherical JointModel;
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
  template<> struct traits<JointDataSpherical> { typedef JointSpherical Joint; };
  template<> struct traits<JointModelSpherical> { typedef JointSpherical Joint; };

  struct JointDataSpherical : public JointDataBase<JointDataSpherical>
  {
    typedef JointSpherical Joint;
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

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }
  }; // struct JointDataSpherical

  struct JointModelSpherical : public JointModelBase<JointModelSpherical>
  {
    typedef JointSpherical Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelSpherical>::id;
    using JointModelBase<JointModelSpherical>::idx_q;
    using JointModelBase<JointModelSpherical>::idx_v;
    using JointModelBase<JointModelSpherical>::lowerPosLimit;
    using JointModelBase<JointModelSpherical>::upperPosLimit;
    using JointModelBase<JointModelSpherical>::maxEffortLimit;
    using JointModelBase<JointModelSpherical>::maxVelocityLimit;
    using JointModelBase<JointModelSpherical>::setIndexes;
    typedef Motion::Vector3 Vector3;
    typedef double Scalar_t;

    JointData createData() const { return JointData(); }

    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());

      const JointData::Quaternion quat(Eigen::Matrix<double,4,1> (q.tail <4> ())); // TODO

      data.M.rotation (quat.matrix());
    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      data.v () = vs.segment<NV> (idx_v ());

      const JointData::Quaternion quat(Eigen::Matrix<double,4,1> (q.tail <4> ())); // TODO
      data.M.rotation (quat.matrix ());
    }
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
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

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      Motion_t::Quaternion_t q(qs.segment<NQ>(idx_q()));
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      Motion_t::Vector3 omega(q_dot);
      Motion_t::Quaternion_t pOmega(se3::exp3(omega));

      Motion_t::Quaternion_t quaternion_result(pOmega*q);
      ConfigVector_t result(quaternion_result.x(),
                            quaternion_result.y(),
                            quaternion_result.z(),
                            quaternion_result.w()
                            );
      return result; 
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2, const double u) const
    { 
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_2 = q2.segment<NQ> (idx_q ());

      Motion_t::Quaternion_t p1 (q_1);
      Motion_t::Quaternion_t p2 (q_2);
      Motion_t::Quaternion_t quaternion_result(p1.slerp(u, p2));

      ConfigVector_t result(quaternion_result.x(),
                            quaternion_result.y(),
                            quaternion_result.z(),
                            quaternion_result.w());

      return result;
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t q(ConfigVector_t::Random());
      q.normalize();
      return q;
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2) const
    { 
      // Compute relative rotation between q2 and q1.
      const int invertor = (q1.segment<NQ> (idx_q ()).dot(q2.segment<NQ> (idx_q ())) < 0 ) ? -1: 1 ;
      Motion_t::Quaternion_t p1 (invertor * q1.segment<NQ>(idx_q()));
      Motion_t::Quaternion_t p2 (q2.segment<NQ>(idx_q()));

      Motion_t::Quaternion_t p (p1*p2.conjugate());
      Eigen::AngleAxis<Scalar_t> angle_axis(p);

      TangentVector_t result(angle_axis.angle() * angle_axis.axis());
      return result;
    } 

    double distance_impl(const Eigen::VectorXd & q1,const Eigen::VectorXd & q2) const
    { 
      return difference_impl(q1, q2).norm();
    }

    JointModelDense<NQ, NV> toDense_impl() const
    {
      return JointModelDense<NQ, NV>( id(),
                                      idx_q(),
                                      idx_v(),
                                      lowerPosLimit(),
                                      upperPosLimit(),
                                      maxEffortLimit(),
                                      maxVelocityLimit()
                                    );
    }

    static const std::string shortname()
    {
      return std::string("JointModelSpherical");
    }

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelSpherical> & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v()
              && jmodel.lowerPosLimit() == lowerPosLimit()
              && jmodel.upperPosLimit() == upperPosLimit()
              && jmodel.maxEffortLimit() == maxEffortLimit()
              && jmodel.maxVelocityLimit() == maxVelocityLimit();
    }
  }; // struct JointModelSpherical

} // namespace se3

#endif // ifndef __se3_joint_spherical_hpp__
