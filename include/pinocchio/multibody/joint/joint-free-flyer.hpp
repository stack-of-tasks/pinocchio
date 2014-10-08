#ifndef __se3_joint_free_flyer_hpp__
#define __se3_joint_free_flyer_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"

namespace se3
{

  struct JointDataFreeFlyer;
  struct JointModelFreeFlyer;

  struct JointFreeFlyer 
  {
    struct BiasZero 
    {
      operator Motion () const { return Motion::Zero(); }
    };
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct ConstraintIdentity
    {
      SE3::Matrix6 se3Action(const SE3 & m) const { return m.toActionMatrix(); }

      struct TransposeConst 
      {
	Force::Vector6 operator* (const Force & phi)
	{  return phi.toVector();  }
      };
      TransposeConst transpose() const { return TransposeConst(); }
      operator ConstraintXd () const { return ConstraintXd(Eigen::MatrixXd::Identity(6,6)); }
    };
    template<typename D>
    friend Motion operator* (const ConstraintIdentity&, const Eigen::MatrixBase<D>& v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,6);
      return Motion(v);
    }

  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Inertia::Matrix6 operator*( const Inertia& Y,const JointFreeFlyer::ConstraintIdentity & )
  {
    return Y.matrix();
  }

  /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
  template<typename D>
  const Eigen::MatrixBase<D> & 
  operator*( const JointFreeFlyer::ConstraintIdentity::TransposeConst &, const Eigen::MatrixBase<D> & F )
  {
    return F;
  }

  namespace internal
  {
    template<>
    struct ActionReturn<JointFreeFlyer::ConstraintIdentity >  
    { typedef SE3::Matrix6 Type; };
  }


  template<>
  struct traits<JointFreeFlyer>
  {
    typedef JointDataFreeFlyer JointData;
    typedef JointModelFreeFlyer JointModel;
    typedef JointFreeFlyer::ConstraintIdentity Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef JointFreeFlyer::BiasZero Bias_t;
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
  };

  struct JointModelFreeFlyer : public JointModelBase<JointModelFreeFlyer>
  {
    typedef JointFreeFlyer Joint;
    SE3_JOINT_TYPEDEF;

    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type q = qs.segment<NQ>(idx_q());
      JointData::Quaternion quat(Eigen::Matrix<double,4,1>(q.tail(4))); // TODO
      data.M = SE3(quat.matrix(),q.head<3>());
    }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type q = qs.segment<NQ>(idx_q());
      data.v = vs.segment<NV>(idx_v());

      JointData::Quaternion quat(Eigen::Matrix<double,4,1>(q.tail(4))); // TODO
      data.M = SE3(quat.matrix(),q.head<3>());
    }
  };

} // namespace se3

#endif // ifndef __se3_joint_free_flyer_hpp__
