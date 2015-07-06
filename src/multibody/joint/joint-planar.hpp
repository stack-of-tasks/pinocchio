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

#ifndef __se3_joint_planar_hpp__
#define __se3_joint_planar_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace se3
{

  struct JointDataPlanar;
  struct JointModelPlanar;

  struct JointPlanar
  {
    struct BiasZero
    {
      operator Motion () const { return Motion::Zero(); }
    }; // struct BiasZero

    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionPlanar
    {
      typedef double Scalar_t;
      typedef MotionTpl<Scalar_t> Motion;

      Scalar_t x_dot_, y_dot_, theta_dot_;

      MotionPlanar () : x_dot_(NAN), y_dot_(NAN), theta_dot_(NAN)      {}
      MotionPlanar (Scalar_t x_dot, Scalar_t y_dot, Scalar_t theta_dot) : x_dot_(x_dot), y_dot_(y_dot), theta_dot_(theta_dot)  {}

      operator Motion() const
      {
        return Motion (Motion::Vector3(x_dot_, y_dot_, 0.), Motion::Vector3(0., 0., theta_dot_));
      }
    }; // struct MotionPlanar

    friend const MotionPlanar operator+ (const MotionPlanar & m, const BiasZero &)
    { return m; }

    friend Motion operator+ (const MotionPlanar & m1, const Motion & m2)
    {
      Motion result (m2);
      result.linear ()[0] += m1.x_dot_;
      result.linear ()[1] += m1.y_dot_;

      result.angular ()[2] += m1.theta_dot_;

      return result;
    }

    struct ConstraintPlanar
    {
    public:
      typedef double Scalar_t;
      typedef MotionTpl<Scalar_t> Motion;
      typedef ForceTpl<Scalar_t> Force;
      typedef Motion::Matrix3 Matrix3;
      typedef Motion::Vector3 Vector3;

    public:

      Motion operator* (const MotionPlanar & vj) const
      { return vj; }


      struct ConstraintTranspose
      {
        const ConstraintPlanar & ref;
        ConstraintTranspose(const ConstraintPlanar & ref) : ref(ref) {}

        Force::Vector3 operator* (const Force & phi)
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

      operator ConstraintXd () const
      {
        Eigen::Matrix<Scalar_t,6,3> S;
        S.block <3,3> (Inertia::LINEAR, 0).setZero ();
        S.block <3,3> (Inertia::ANGULAR, 0).setZero ();
        S(Inertia::LINEAR + 0,0) = 1.;
        S(Inertia::LINEAR + 1,1) = 1.;
        S(Inertia::ANGULAR + 2,2) = 1.;
        return ConstraintXd(S);
      }

      Eigen::Matrix <Scalar_t,6,3> se3Action (const SE3 & m) const
      {
        Eigen::Matrix <double,6,3> X_subspace;
        X_subspace.block <3,2> (Motion::LINEAR, 0) = skew (m.translation ()) * m.rotation ().leftCols <2> ();
        X_subspace.block <3,1> (Motion::LINEAR, 2) = m.rotation ().rightCols <1> ();

        X_subspace.block <3,2> (Motion::ANGULAR, 0) = m.rotation ().leftCols <2> ();
        X_subspace.block <3,1> (Motion::ANGULAR, 2).setZero ();

        return X_subspace;
      }

    }; // struct ConstraintPlanar

    template<typename D>
    friend Motion operator* (const ConstraintPlanar &, const Eigen::MatrixBase<D> & v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
      Motion result (Motion::Zero ());
      result.linear ().template head<2> () = v.template topRows<2> ();
      result.angular ().template tail<1> () = v.template bottomRows<1> ();
      return result;
    }

  }; // struct JointPlanar

  Motion operator^ (const Motion & m1, const JointPlanar::MotionPlanar & m2)
  {
    Motion result;

    const Motion::Vector3 & m1_t = m1.linear();
    const Motion::Vector3 & m1_w = m1.angular();

    result.angular () << m1_w(1) * m2.theta_dot_, - m1_w(0) * m2.theta_dot_, 0.;
    result.linear () << m1_t(1) * m2.theta_dot_ - m1_w(2) * m2.y_dot_, - m1_t(0) * m2.theta_dot_ + m1_w(2) * m2.x_dot_, m1_w(0) * m2.y_dot_ - m1_w(1) * m2.x_dot_;

    return result;
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix <Inertia::Scalar_t, 6, 3> operator* (const Inertia & Y, const JointPlanar::ConstraintPlanar &)
  {
    Eigen::Matrix <Inertia::Scalar_t, 6, 3> M;
    const double mass = Y.mass ();
    const Inertia::Vector3 & com = Y.lever ();
    const Symmetric3 & inertia = Y.inertia ();

    M.topLeftCorner <3,3> ().setZero ();
    M.topLeftCorner <2,2> ().diagonal ().fill (mass);

    Inertia::Vector3 mc (mass * com);
    M.rightCols <1> ().head <2> () << mc(1), - mc(0);

    M.bottomLeftCorner <3,2> () << 0., mc(2), - mc(1), 0., mc(1), -mc(0);
    M.rightCols <1> ().tail <3> () = inertia.data ().tail <3> ();
    M.rightCols <1> ().head <2> () = mc.head <2> () * com(2);
    M.rightCols <1> ().tail <1> () = -mc.head <2> ().transpose () * com.head <2> ();

    return M;
  }

  namespace internal
  {
    template<>
    struct ActionReturn<JointPlanar::ConstraintPlanar >
    { typedef Eigen::Matrix<double,6,3> Type; };
  }

  template<>
  struct traits<JointPlanar>
  {
    typedef JointDataPlanar JointData;
    typedef JointModelPlanar JointModel;
    typedef JointPlanar::ConstraintPlanar Constraint_t;
    typedef SE3 Transformation_t;
    typedef JointPlanar::MotionPlanar Motion_t;
    typedef JointPlanar::BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,3> F_t;
    enum {
      NQ = 3,
      NV = 3
    };
  };
  template<> struct traits<JointDataPlanar> { typedef JointPlanar Joint; };
  template<> struct traits<JointModelPlanar> { typedef JointPlanar Joint; };

  struct JointDataPlanar : public JointDataBase<JointDataPlanar>
  {
    typedef JointPlanar Joint;
    SE3_JOINT_TYPEDEF;

    typedef Motion::Scalar Scalar;

    typedef Eigen::Matrix<Scalar,6,6> Matrix6;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    JointDataPlanar () : M(1) {}
  };

  struct JointModelPlanar : public JointModelBase<JointModelPlanar>
  {
    typedef JointPlanar Joint;
    SE3_JOINT_TYPEDEF;

    JointData createData() const { return JointData(); }

    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ>(idx_q ());

      double c_theta,s_theta; SINCOS (q(2), &s_theta, &c_theta);

      data.M.rotation ().topLeftCorner <2,2> () << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation ().head <2> () = q.head<2> ();

    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NQ> (idx_v ());

      double c_theta,s_theta; SINCOS (q(2), &s_theta, &c_theta);

      data.M.rotation ().topLeftCorner <2,2> () << c_theta, -s_theta, s_theta, c_theta;
      data.M.translation ().head <2> () = q.head<2> ();

      data.v.x_dot_ = q_dot(0);
      data.v.y_dot_ = q_dot(1);
      data.v.theta_dot_ = q_dot(2);
    }
  };

} // namespace se3

#endif // ifndef __se3_joint_planar_hpp__
