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

#ifndef __se3_joint_free_flyer_hpp__
#define __se3_joint_free_flyer_hpp__

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"

namespace se3
{

  struct JointDataFreeFlyer;
  struct JointModelFreeFlyer;

  struct ConstraintIdentity;

  template <>
  struct traits< ConstraintIdentity >
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
  }; // traits ConstraintRevolute


    struct ConstraintIdentity : ConstraintBase < ConstraintIdentity >
    {
      SPATIAL_TYPEDEF_NO_TEMPLATE(ConstraintIdentity);
      enum { NV = 6, Options = 0 };
      typedef traits<ConstraintIdentity>::JointMotion JointMotion;
      typedef traits<ConstraintIdentity>::JointForce JointForce;
      typedef traits<ConstraintIdentity>::DenseBase DenseBase;

      SE3::Matrix6 se3Action(const SE3 & m) const { return m.toActionMatrix(); }

      int nv_impl() const { return NV; }

      struct TransposeConst 
      {
        Force::Vector6 operator* (const Force & phi)
        {  return phi.toVector();  }
      };
      
      TransposeConst transpose() const { return TransposeConst(); }
      operator ConstraintXd () const { return ConstraintXd(SE3::Matrix6::Identity()); }
    }; // struct ConstraintIdentity

    template<typename D>
    Motion operator* (const ConstraintIdentity&, const Eigen::MatrixBase<D>& v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,6);
      return Motion(v);
    }


  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Inertia::Matrix6 operator*( const Inertia& Y,const ConstraintIdentity & )
  {
    return Y.matrix();
  }

  /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
  template<typename D>
  const Eigen::MatrixBase<D> & 
  operator*( const ConstraintIdentity::TransposeConst &, const Eigen::MatrixBase<D> & F )
  {
    return F;
  }

  namespace internal
  {
    template<>
    struct ActionReturn<ConstraintIdentity >  
    { typedef SE3::Matrix6 Type; };
  }

  struct JointFreeFlyer;

  template<>
  struct traits<JointFreeFlyer>
  {
    typedef JointDataFreeFlyer JointData;
    typedef JointModelFreeFlyer JointModel;
    typedef ConstraintIdentity Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,6> F_t;
    enum {
      NQ = 7,
      NV = 6
    };
  };
  template<> struct traits<JointDataFreeFlyer> { typedef JointFreeFlyer Joint; };
  template<> struct traits<JointModelFreeFlyer> { typedef JointFreeFlyer Joint; };

  struct JointDataFreeFlyer : public JointDataBase<JointDataFreeFlyer>
  {
    typedef JointFreeFlyer Joint;
    SE3_JOINT_TYPEDEF;

    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Quaternion<double> Quaternion;
    typedef Eigen::Matrix<double,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F; // TODO if not used anymore, clean F_t
    JointDataFreeFlyer() : M(1)
    {
    }

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }
  }; // struct JointDataFreeFlyer

  struct JointModelFreeFlyer : public JointModelBase<JointModelFreeFlyer>
  {
    typedef JointFreeFlyer Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelFreeFlyer>::id;
    using JointModelBase<JointModelFreeFlyer>::idx_q;
    using JointModelBase<JointModelFreeFlyer>::idx_v;
    using JointModelBase<JointModelFreeFlyer>::lowerPosLimit;
    using JointModelBase<JointModelFreeFlyer>::upperPosLimit;
    using JointModelBase<JointModelFreeFlyer>::maxEffortLimit;
    using JointModelBase<JointModelFreeFlyer>::maxVelocityLimit;
    using JointModelBase<JointModelFreeFlyer>::setIndexes;

    JointData createData() const { return JointData(); }
    void calc( JointData& data,
         const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type q = qs.segment<NQ>(idx_q());
      const JointData::Quaternion quat(Eigen::Matrix<double,4,1> (q.tail <4> ())); // TODO

      data.M.rotation (quat.matrix());
      data.M.translation (q.head<3>());
    }
    void calc( JointData& data, 
         const Eigen::VectorXd & qs, 
         const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type q = qs.segment<NQ>(idx_q());
      data.v = vs.segment<NV>(idx_v());

      const JointData::Quaternion quat(Eigen::Matrix<double,4,1> (q.tail <4> ())); // TODO
//      data.M = SE3(quat.matrix(),q.head<3>());
      data.M.rotation (quat.matrix());
      data.M.translation (q.head<3>());
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
      return std::string("JointModelFreeFlyer");
    }

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelFreeFlyer> & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v()
              && jmodel.lowerPosLimit() == lowerPosLimit()
              && jmodel.upperPosLimit() == upperPosLimit()
              && jmodel.maxEffortLimit() == maxEffortLimit()
              && jmodel.maxVelocityLimit() == maxVelocityLimit();
    }
  }; // struct JointModelFreeFlyer

} // namespace se3

#endif // ifndef __se3_joint_free_flyer_hpp__
