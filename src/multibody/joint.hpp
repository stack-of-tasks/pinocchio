#ifndef __se3_joint_hpp__
#define __se3_joint_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/variant.hpp>

namespace se3
{
  template<class C> struct traits {};

#define SE3_JOINT_TYPEDEF \
  typedef typename traits<Joint>::JointData JointData; \
  typedef typename traits<Joint>::JointModel JointModel; \
  typedef typename traits<Joint>::Constraint_t Constraint_t; \
  typedef typename traits<Joint>::Transformation_t Transformation_t; \
  typedef typename traits<Joint>::Velocity_t Velocity_t; \
  typedef typename traits<Joint>::Bias_t Bias_t; \
  typedef typename traits<Joint>::JointMotion_t JointMotion_t; \
  enum { \
    nq = traits<Joint>::nq, \
    nv = traits<Joint>::nv \
  }

  template<typename _JointData>
  struct JointDataBase
  {
    typedef typename traits<_JointData>::Joint Joint;
    SE3_JOINT_TYPEDEF;

    JointData& derived() { return *static_cast<JointData*>(this); }
    const JointData& derived() const { return *static_cast<const JointData*>(this); }

    const Constraint_t     & S()   { return static_cast<JointData*>(this)->S;   }
    const Transformation_t & M()   { return static_cast<JointData*>(this)->M;   }
    const Velocity_t       & v()   { return static_cast<JointData*>(this)->v;   }
    const Bias_t           & c()   { return static_cast<JointData*>(this)->c;   }
    const JointMotion_t    & qdd() { return static_cast<JointData*>(this)->qdd; }
  };



  template<typename _JointModel>
  struct JointModelBase
  {
    typedef typename traits<_JointModel>::Joint Joint;
    SE3_JOINT_TYPEDEF;

    JointModel& derived() { return *static_cast<JointModel*>(this); }
    const JointModel& derived() const { return *static_cast<const JointModel*>(this); }

    JointData createData() const { return static_cast<const JointModel*>(this)->createData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    { return static_cast<const JointModel*>(this)->calc(data,qs,vs,as); }

  private:
    int i_q,i_v;
  public:
    const int & idx_q() const { return i_q; }
    const int & idx_v() const { return i_v; }
    void setIndexes(int q,int v) { i_q = q; i_v = v; }
  };



  struct JointDataFreeFlyer;
  struct JointModelFreeFlyer;



  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */

  struct JointDataRX;
  struct JointModelRX;

  struct JointRX {
    struct BiasZero {};
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }
  };

  template<>
  struct traits<JointRX>
  {
    typedef JointDataRX JointData;
    typedef JointModelRX JointModel;
    typedef ConstraintTpl<1,double,0> Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Velocity_t;
    typedef JointRX::BiasZero Bias_t;
    typedef Constraint_t::JointMotion JointMotion_t;
    enum {
      nq = 1,
      nv = 1
    };
  };

  template<> struct traits<JointDataRX> { typedef JointRX Joint; };
  template<> struct traits<JointModelRX> { typedef JointRX Joint; };

  struct JointDataRX : public JointDataBase<JointDataRX>
  {
    typedef JointRX Joint;
    SE3_JOINT_TYPEDEF;

    Constraint_t S;
    Transformation_t M;
    Velocity_t v;
    Bias_t c;
    JointMotion_t qdd;

    JointDataRX() : S(),M(1)
    {
      S.matrix() << 0,0,0,1,0,0;
      M.translation(SE3::Vector3::Zero());
      v.linear(Motion::Vector3::Zero());
    }
  };

  struct JointModelRX : public JointModelBase<JointModelRX>
  {
    typedef JointRX Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRX>::idx_q;
    using JointModelBase<JointModelRX>::idx_v;
    using JointModelBase<JointModelRX>::setIndexes;
    
    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];
      data.qdd[0] = as[idx_v()];

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

  struct JointFreeFlyer 
  {
    struct BiasZero {};
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct ConstraintIdentity
    {
      const ConstraintIdentity& transpose() const { return *this; }
    };
    template<typename D>
    friend Motion operator* (const ConstraintIdentity&, const Eigen::MatrixBase<D>& v)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,6);
      return Motion(v);
    }

    friend Force::Vector6 operator* (const ConstraintIdentity&, const Force & phi)
    {  return phi.toVector();  }
  };

  struct JointModelFreeFlyer;
  struct JointDataFreeFlyer;

  template<>
  struct traits<JointFreeFlyer>
  {
    typedef JointDataFreeFlyer JointData;
    typedef JointModelFreeFlyer JointModel;
    typedef JointFreeFlyer::ConstraintIdentity Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Velocity_t;
    typedef JointFreeFlyer::BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,1> JointMotion_t;
    enum {
      nq = 7,
      nv = 6
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
    Velocity_t v;
    Bias_t c;
    JointMotion_t qdd;

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
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs, 
	       const Eigen::VectorXd & as ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<nq>::Type q = qs.segment<nq>(idx_q());
      data.v = vs .segment<nv>(idx_v());
      data.qdd = as.segment<nv>(idx_v());

      JointData::Quaternion quat(Eigen::Matrix<double,4,1>(q.tail(4))); // TODO
      data.M = SE3(quat.matrix(),q.head<3>());
    }
  };

} // namespace se3

namespace se3
{
  /* --- VARIANT ------------------------------------------------------------ */
  /* --- VARIANT ------------------------------------------------------------ */
  /* --- VARIANT ------------------------------------------------------------ */

  typedef boost::variant< JointModelRX,JointModelFreeFlyer> JointModelVariant;
  typedef boost::variant< JointDataRX,JointDataFreeFlyer> JointDataVariant;

  typedef std::vector<JointModelVariant> JointModelVector;
  typedef std::vector<JointDataVariant> JointDataVector;

  class CreateJointData: public boost::static_visitor<JointDataVariant>
  {
  public:
    template<typename D>
    JointDataVariant operator()(const JointModelBase<D> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( CreateJointData(), jmodel ); }
  };

} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointModelVariant);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointDataVariant);

#endif // ifndef __se3_joint_hpp__
