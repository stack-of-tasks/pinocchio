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

#ifndef __se3_joint_translation_hpp__
#define __se3_joint_translation_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

#include <stdexcept>

namespace se3
{

  struct JointDataTranslation;
  struct JointModelTranslation;

  struct MotionTranslation;
  template <>
  struct traits < MotionTranslation >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
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
    enum {
      NQ = 3,
      NV = 3
    };
    typedef double Scalar;
    typedef JointDataTranslation JointDataDerived;
    typedef JointModelTranslation JointModelDerived;
    typedef ConstraintTranslationSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionTranslation Motion_t;
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

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());


      return (q + q_dot);
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    { 
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_0 = q0.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());

      return ((1-u) * q_0 + u * q_1);
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t result(ConfigVector_t::Random());
      return result;
    }

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & lower_pos_limit, const ConfigVector_t & upper_pos_limit ) const throw (std::runtime_error)
    { 
      ConfigVector_t result;
      for (int i = 0; i < result.size(); ++i)
      {
        if(lower_pos_limit[i] == -std::numeric_limits<double>::infinity() || 
            upper_pos_limit[i] == std::numeric_limits<double>::infinity() )
        {
          std::ostringstream error;
          error << "non bounded limit. Cannot uniformly sample joint nb " << id() ;
          assert(false && "non bounded limit. Cannot uniformly sample joint translation");
          throw std::runtime_error(error.str());
        }
        result[i] = lower_pos_limit[i] + ( upper_pos_limit[i] - lower_pos_limit[i]) * rand()/RAND_MAX;
      }
      return result;
    }  

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_0 = q0.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());

      return q_1-q_0;
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return difference_impl(q0,q1).norm();
    }
    
    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t q;
      q << 0,0,0;
      return q;
    } 

    bool isSameConfiguration_impl(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_1 = q1.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q_2 = q2.segment<NQ> (idx_q ());

      return q_1.isApprox(q_2, prec);
    } 
    
    static std::string classname() { return std::string("JointModelTranslation"); }
    std::string shortname() const { return classname(); }

  }; // struct JointModelTranslation
  
} // namespace se3

#endif // ifndef __se3_joint_translation_hpp__
