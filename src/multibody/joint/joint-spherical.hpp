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

#ifndef __se3_joint_spherical_hpp__
#define __se3_joint_spherical_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  struct JointDataSpherical;
  struct JointModelSpherical;

  struct JointSpherical
  {
    struct BiasZero
    {
      operator Motion () const { return Motion::Zero(); }
    }; // struct BiasZero

    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionSpherical
    {
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

    friend const MotionSpherical operator+ (const MotionSpherical & m, const BiasZero & )
    { return m; }

    friend Motion operator+ (const MotionSpherical & m1, const Motion & m2)
    {
      return Motion( m2.linear(), m2.angular() + m1.w);
    }

    struct ConstraintRotationalSubspace
    {

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
    friend Motion operator* (const ConstraintRotationalSubspace&, const Eigen::MatrixBase<D>& v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,3);
      return Motion (Motion::Vector3::Zero (), v);
    }

  }; // struct JointSpherical

  Motion operator^ (const Motion & m1, const JointSpherical::MotionSpherical & m2)
  {
    return Motion(m1.linear ().cross (m2.w), m1.angular ().cross (m2.w));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix <double, 6, 3> operator* (const Inertia & Y, const JointSpherical::ConstraintRotationalSubspace & )
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
    struct ActionReturn<JointSpherical::ConstraintRotationalSubspace >
    { typedef Eigen::Matrix<double,6,3> Type; };
  }


  template<>
  struct traits<JointSpherical>
  {
    typedef JointDataSpherical JointData;
    typedef JointModelSpherical JointModel;
    typedef JointSpherical::ConstraintRotationalSubspace Constraint_t;
    typedef SE3 Transformation_t;
    typedef JointSpherical::MotionSpherical Motion_t;
    typedef JointSpherical::BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,3> F_t;
    enum {
      NQ = 4,
      NV = 3
    };
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

    JointDataSpherical () : M(1)
    {}
  };

  struct JointModelSpherical : public JointModelBase<JointModelSpherical>
  {
    typedef JointSpherical Joint;
    SE3_JOINT_TYPEDEF;

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
  };

} // namespace se3

#endif // ifndef __se3_joint_spherical_hpp__
