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

    const Constraint_t     & S() { return static_cast<JointData*>(this)->S; }
    const Transformation_t & M() { return static_cast<JointData*>(this)->M; };
    const Velocity_t       & v() { return static_cast<JointData*>(this)->v; };
    const Bias_t           & c() { return static_cast<JointData*>(this)->c; };
  };



  template<typename Joint>
  struct JointBase
  {
    typedef typename traits<Joint>::JointData JointData;
    typedef typename traits<Joint>::Placement_t Placement_t;
    typedef typename traits<Joint>::Configuration_t Configuration_t;
    typedef typename traits<Joint>::Velocity_t Velocity;

    JointData createData() { return static_cast<Joint*>(this)->createData(); }
  };



  // struct JointRXData;
  // template<>
  // struct traits<JointRXData>
  // {
  //   typedef Eigen::Matrix<double,6,1> Constraint_t;
  //   typedef se3::SE3 Transformation_t;
  //   typedef Eigen::Matrix<double,6,1> Velocity_t;
  //   typedef BiasZero Bias_t;
  // };

  // struct JointRXData : public JointDataBase<JointRXData>
  // {
  //   typedef typename traits<JointRXData>::Constraint_t Constraint_t;
  //   typedef typename traits<JointRXData>::Transformation_t Transformation_t;
  //   typedef typename traits<JointRXData>::Velocity_t Velocity_t;
  //   typedef typename traits<JointRXData>::Bias_t Bias_t;

  //   Constraint_t S;
  //   Transformation_t M;
  //   Velocity_t v;
  //   Bias_t c;

  //   JointRXData() { S << 0,0,0,1,0,0; }
  // };

  // struct JointRX;
  // template<>
  // struct traits<JointRX>
  // {
  //   typedef JointRXData JointData;
  //   typedef SE3 Placement_t;
  //   typedef double Configuration_t;
  //   typedef double Velocity_t;
  // };

  // struct JointRX : public JointBase<JointRX>
  // {
  //   typedef traits<JointRX>::JointData JointData;

  //   JointData createData() const { return JointData(); }
  //   void calc( JointData& data, Configuration_t q, Configuration_t v )
  //   {



  //   }
  // };

  struct JointDataRX;
  struct JointModelRX;

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
  };

  struct JointDataRX
  {
    typedef typename traits<JointDataRX>::Constraint_t Constraint_t;
    typedef typename traits<JointDataRX>::Transformation_t Transformation_t;
    typedef typename traits<JointDataRX>::Velocity_t Velocity_t;
    typedef typename traits<JointDataRX>::Bias_t Bias_t;
    typedef typename traits<JointDataRX>::JointModel JointModel;
    //typedef typename traits<JointDataRX>::;

    Constraint_t S;
    Transformation_t M;
    Velocity_t v;
    Bias_t c;

    JointDataRX() : M(1)
   {
      S << 0,0,0,1,0,0;
    }
  };

  template<>
  struct traits<JointModelRX>
  {
    typedef JointDataRX JointData;
    typedef SE3 Placement_t;
  };

  struct JointModelRX
  {
    typedef traits<JointModelRX>::JointData JointData;

    static const int nq = 1;
    static const int nv = 1;
    int idx_q,idx_v;

    JointModelRX() : idx_q(-1),idx_v(-1) {} // Default constructor for std::vector
    JointModelRX( int index_q,int index_v ) : idx_q(index_q),idx_v(index_v) {}
    JointModelRX( int index_q,int index_v, const JointModelRX& ) : idx_q(index_q),idx_v(index_v) {}

    JointData createData() const { return JointData(); }
    void calc( JointData& data, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs )
    {
      const double & q = qs[idx_q];
      const double & v = vs[idx_v];
      
      //data.M.rotation(  Eigen::AngleAxis<double>(q, Eigen::Vector3d::UnitX()).matrix() );
      double ca,sa; sincos(q,&sa,&ca);
      Eigen::Matrix3d R3; 
      R3 << 
	1,0,0,
	0,ca,sa,
	0,-sa,ca;
      data.M.rotation(R3);
      Eigen::Vector3d v3; v3 << v,0,0; 
      data.v.angular(v3);
    }
  };




} // namespace se3

#endif // ifndef __se3_joint_hpp__
