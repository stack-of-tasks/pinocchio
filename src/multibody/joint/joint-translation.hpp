//
// Copyright (c) 2015 CNRS
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

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-dense.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  struct JointDataTranslation;
  struct JointModelTranslation;

  struct MotionTranslation;
  template <>
  struct traits < MotionTranslation >
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
  }; // traits MotionTranslation

  struct MotionTranslation : MotionBase < MotionTranslation >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(MotionTranslation);

    MotionTranslation ()                   : v (Motion::Vector3 (NAN, NAN, NAN)) {}
    MotionTranslation (const Motion::Vector3 & v) : v (v)  {}
    MotionTranslation (const MotionTranslation & other) : v (other.v)  {}
    Vector3 v;

    Vector3 & operator() () { return v; }
    const Vector3 & operator() () const { return v; }

    operator Motion() const
    {
      return Motion (v, Motion::Vector3::Zero ());
    }

    MotionTranslation & operator= (const MotionTranslation & other)
    {
      v = other.v;
      return *this;
    }
  }; // struct MotionTranslation

  inline const MotionTranslation operator+ (const MotionTranslation & m, const BiasZero &)
  { return m; }

  inline Motion operator+ (const MotionTranslation & m1, const Motion & m2)
  {
    return Motion (m2.linear () + m1.v, m2.angular ());
  }

  struct ConstraintTranslationSubspace;
  template <>
  struct traits < ConstraintTranslationSubspace>
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
    typedef Eigen::Matrix<Scalar_t,1,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar_t,1,1,0> JointForce;
    typedef Eigen::Matrix<Scalar_t,6,1> DenseBase;
  }; // traits ConstraintTranslationSubspace

  struct ConstraintTranslationSubspace : ConstraintBase < ConstraintTranslationSubspace >
  {
    SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintTranslationSubspace);
    enum { NV = 3, Options = 0 };
    typedef traits<ConstraintTranslationSubspace>::JointMotion JointMotion;
    typedef traits<ConstraintTranslationSubspace>::JointForce JointForce;
    typedef traits<ConstraintTranslationSubspace>::DenseBase DenseBase;
    ConstraintTranslationSubspace () {}

    Motion operator* (const MotionTranslation & vj) const
    { return Motion (vj (), Motion::Vector3::Zero ()); }

    int nv_impl() const { return NV; }

    struct ConstraintTranspose
    {
      const ConstraintTranslationSubspace & ref;
      ConstraintTranspose(const ConstraintTranslationSubspace & ref) : ref(ref) {}

      Force::Vector3 operator* (const Force & phi)
      {
        return phi.linear ();
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

    operator ConstraintXd () const
    {
      Eigen::Matrix<double,6,3> S;
      S.block <3,3> (Inertia::LINEAR, 0).setIdentity ();
      S.block <3,3> (Inertia::ANGULAR, 0).setZero ();
      return ConstraintXd(S);
    }

    Eigen::Matrix <double,6,3> se3Action (const SE3 & m) const
    {
      Eigen::Matrix <double,6,3> M;
      M.block <3,3> (Motion::LINEAR, 0) = m.rotation ();
      M.block <3,3> (Motion::ANGULAR, 0).setZero ();

      return M;
    }

  }; // struct ConstraintTranslationSubspace

  template<typename D>
  Motion operator* (const ConstraintTranslationSubspace &, const Eigen::MatrixBase<D> & v)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
    return Motion (v, Motion::Vector3::Zero ());
  }


  inline Motion operator^ (const Motion & m1, const MotionTranslation & m2)
  {
    return Motion (m1.angular ().cross (m2.v), Motion::Vector3::Zero ());
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
    struct ActionReturn<ConstraintTranslationSubspace >
    { typedef Eigen::Matrix<double,6,3> Type; };
  }


  struct JointTranslation;
  template<>
  struct traits<JointTranslation>
  {
    typedef JointDataTranslation JointData;
    typedef JointModelTranslation JointModel;
    typedef ConstraintTranslationSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionTranslation Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,3> F_t;
    enum {
      NQ = 3,
      NV = 3
    };
  }; // traits JointTranslation
  
  template<> struct traits<JointDataTranslation> { typedef JointTranslation Joint; };
  template<> struct traits<JointModelTranslation> { typedef JointTranslation Joint; };

  struct JointDataTranslation : public JointDataBase<JointDataTranslation>
  {
    typedef JointTranslation Joint;
    SE3_JOINT_TYPEDEF;

    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Matrix<double,3,3> Matrix3;
    typedef Eigen::Matrix<double,3,1> Vector3;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t

    JointDataTranslation () : M(1) {}

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }
  }; // struct JointDataTranslation

  struct JointModelTranslation : public JointModelBase<JointModelTranslation>
  {
    typedef JointTranslation Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelTranslation>::id;
    using JointModelBase<JointModelTranslation>::idx_q;
    using JointModelBase<JointModelTranslation>::idx_v;
    using JointModelBase<JointModelTranslation>::lowerPosLimit;
    using JointModelBase<JointModelTranslation>::upperPosLimit;
    using JointModelBase<JointModelTranslation>::maxEffortLimit;
    using JointModelBase<JointModelTranslation>::maxVelocityLimit;
    using JointModelBase<JointModelTranslation>::setIndexes;

    JointData createData() const { return JointData(); }

    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      data.M.translation (qs.segment<NQ>(idx_q ()));
    }
    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      data.M.translation (qs.segment<NQ> (idx_q ()));
      data.v () = vs.segment<NQ> (idx_v ());
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

    bool operator == (const JointModelTranslation& /*Ohter*/) const
    {
      return true; // TODO ?? used to bind variant in python
    }
  }; // struct JointModelTranslation
  
} // namespace se3

#endif // ifndef __se3_joint_translation_hpp__
