#ifndef __se3_joint_hpp__
#define __se3_joint_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"
#include <Eigen/Geometry>

namespace se3
{
  template<class C>
  struct traits {};

  template<typename JointData>
  struct JointDataBase
  {
    typedef typename traits<JointData>::Constraint_t Constraint_t;
    typedef typename traits<JointData>::Transformation_t Transformation_t;
    typedef typename traits<JointData>::Velocity_t Velocity_t;
    typedef typename traits<JointData>::Bias_t Bias_t;
    typedef typename traits<JointData>::JointMotion_t JointMotion_t;

    JointData& derived() { return *static_cast<JointData*>(this); }
    const JointData& derived() const { return *static_cast<JointData*>(this); }

    const Constraint_t     & S()   { return static_cast<JointData*>(this)->S;   }
    const Transformation_t & M()   { return static_cast<JointData*>(this)->M;   }
    const Velocity_t       & v()   { return static_cast<JointData*>(this)->v;   }
    const Bias_t           & c()   { return static_cast<JointData*>(this)->c;   }
    const JointMotion_t    & qdd() { return static_cast<JointData*>(this)->qdd; }
  };



  template<typename Joint>
  struct JointModelBase
  {
    typedef typename traits<Joint>::JointData JointData;

    JointData createData() const { return static_cast<const Joint*>(this)->createData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    { return static_cast<const Joint*>(this)->calc(data,qs,vs,as); }

    int idx_q() const { return static_cast<const Joint *>(this)->idx_q; }
    int idx_v() const { return static_cast<const Joint *>(this)->idx_v; }
    int nq()    const { return static_cast<const Joint *>(this)->nq;    }
    int nv()    const { return static_cast<const Joint *>(this)->nv;    }
  };


  struct JointDataRX;
  struct JointModelRX;

  struct JointDataFreeFlyer;
  struct JointModelFreeFlyer;



  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */

  template<>
  struct traits<JointDataRX>
  {
    struct BiasZero {};
    template<typename D>
    friend const Eigen::MatrixBase<D> & operator+ ( const BiasZero&, const Eigen::MatrixBase<D>& v) { return v; }
    template<typename D>
    friend const Eigen::MatrixBase<D> & operator+ ( const Eigen::MatrixBase<D>& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
   
    typedef Eigen::Matrix<double,6,1> Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Velocity_t;
    typedef BiasZero Bias_t;
    typedef JointModelRX JointModel;
    typedef Eigen::Matrix<double,1,1> JointMotion_t;
  };

  struct JointDataRX : public JointDataBase<JointDataRX>
  {
    typedef typename traits<JointDataRX>::Constraint_t Constraint_t;
    typedef typename traits<JointDataRX>::Transformation_t Transformation_t;
    typedef typename traits<JointDataRX>::Velocity_t Velocity_t;
    typedef typename traits<JointDataRX>::Bias_t Bias_t;
    typedef typename traits<JointDataRX>::JointMotion_t JointMotion_t;
    typedef typename traits<JointDataRX>::JointModel JointModel;

    Constraint_t S;
    Transformation_t M;
    Velocity_t v;
    Bias_t c;
    JointMotion_t qdd;

    JointDataRX() : M(1)
   {
      S << 0,0,0,1,0,0;
      M.translation(SE3::Vector3::Zero());
      v.linear(Motion::Vector3::Zero());
    }
  };

  template<>
  struct traits<JointModelRX>
  {
    typedef JointDataRX JointData;
    typedef SE3 Placement_t;
  };

  struct JointModelRX : public JointModelBase<JointModelRX>
  {
    typedef traits<JointModelRX>::JointData JointData;

    static const int nq = 1;
    static const int nv = 1;
    int idx_q,idx_v;

    JointModelRX() : idx_q(-1),idx_v(-1) {} // Default constructor for std::vector
    JointModelRX( int index_q,int index_v ) : idx_q(index_q),idx_v(index_v) {}

    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    {
      const double & q = qs[idx_q];
      const double & v = vs[idx_v];
      data.qdd[0] = as[idx_q];

      data.M.rotation(rotationX(q));
      data.v.angular(Eigen::Vector3d(v,0,0));
    }

    static inline Eigen::Matrix3d rotationX(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	1,0,0,
	0,ca,sa,
	0,-sa,ca;
      return R3;
    }

  };



  /* --- REVOLUTE FF -------------------------------------------------------- */
  /* --- REVOLUTE FF -------------------------------------------------------- */
  /* --- REVOLUTE FF -------------------------------------------------------- */

  template<>
  struct traits<JointDataFreeFlyer>
  {
    struct BiasZero {};
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
   
    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Quaternion<double> Quaternion;
    typedef Eigen::Matrix<double,3,1> Vector3;

    typedef Matrix6 Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Velocity_t;
    typedef BiasZero Bias_t;
    typedef JointModelFreeFlyer JointModel;
    typedef Eigen::Matrix<double,6,1> JointMotion_t;
  };

  struct JointDataFreeFlyer : public JointDataBase<JointDataFreeFlyer>
  {
    typedef typename traits<JointDataFreeFlyer>::Constraint_t Constraint_t;
    typedef typename traits<JointDataFreeFlyer>::Transformation_t Transformation_t;
    typedef typename traits<JointDataFreeFlyer>::Velocity_t Velocity_t;
    typedef typename traits<JointDataFreeFlyer>::Bias_t Bias_t;
    typedef typename traits<JointDataFreeFlyer>::JointMotion_t JointMotion_t;
    typedef typename traits<JointDataFreeFlyer>::JointModel JointModel;

    typedef Eigen::Matrix<double,6,6> Matrix6;
    typedef Eigen::Quaternion<double> Quaternion;
    typedef Eigen::Matrix<double,3,1> Vector3;
    
    Constraint_t S;
    Transformation_t M;
    Velocity_t v;
    Bias_t c;
    JointMotion_t qdd;

    Quaternion rotation;

    JointDataFreeFlyer() : M(1)
    {
     S = Eigen::Matrix<double,6,6>::Identity();
    }
  };

  template<>
  struct traits<JointModelFreeFlyer>
  {
    typedef JointDataFreeFlyer JointData;
  };

  struct JointModelFreeFlyer : public JointModelBase<JointModelFreeFlyer>
  {
    typedef traits<JointModelFreeFlyer>::JointData JointData;

    static const int nq = 7;
    static const int nv = 6;
    int idx_q,idx_v;

    JointModelFreeFlyer() : idx_q(-1),idx_v(-1) {} // Default constructor for std::vector
    JointModelFreeFlyer( int index_q,int index_v ) : idx_q(index_q),idx_v(index_v) {}

    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<nq>::Type q = qs.segment<nq>(idx_q);
      data.v = vs .segment<nv>(idx_v);
      data.qdd = as.segment<nv>(idx_v);

      JointData::Quaternion quat(Eigen::Matrix<double,4,1>(q.tail(4)));
      data.M = SE3(quat.matrix(),q.head<3>());
    }
  };

} // namespace se3

#endif // ifndef __se3_joint_hpp__
