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

  /*
   * *** FORWARD ***
   * J::calc()
   * SE3    = SE3 * J::SE3
   * Motion = J::Motion
   * Motion = J::Constraint*J::JointMotion + J::Bias + Motion^J::Motion
   * Force  = Inertia*Motion  + Inertia.vxiv(Motion)
   *
   * *** BACKWARD *** 
   * J::JointForce = J::Constraint::Transpose*J::Force
   */

#define SE3_JOINT_TYPEDEF \
  typedef typename traits<Joint>::JointData JointData; \
  typedef typename traits<Joint>::JointModel JointModel; \
  typedef typename traits<Joint>::Constraint_t Constraint_t; \
  typedef typename traits<Joint>::Transformation_t Transformation_t; \
  typedef typename traits<Joint>::Motion_t Motion_t; \
  typedef typename traits<Joint>::Bias_t Bias_t; \
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
    const Motion_t       & v()   { return static_cast<JointData*>(this)->v;   }
    const Bias_t           & c()   { return static_cast<JointData*>(this)->c;   }
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
	       const Eigen::VectorXd & vs ) const
    { return static_cast<const JointModel*>(this)->calc(data,qs,vs); }

  private:
    int i_q,i_v;
  public:
    const int & idx_q() const { return i_q; }
    const int & idx_v() const { return i_v; }
    void setIndexes(int q,int v) { i_q = q; i_v = v; }

    template<typename D>
    typename D::template ConstFixedSegmentReturnType<nv>::Type jointMotion(const Eigen::MatrixBase<D>& a) const     { return a.template segment<nv>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<nv>::Type jointMotion(Eigen::MatrixBase<D>& a) const 
    { return a.template segment<nv>(i_v); }
    template<typename D>
    typename D::template ConstFixedSegmentReturnType<nv>::Type jointForce(const Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<nv>(i_v); }
    template<typename D>
    typename D::template FixedSegmentReturnType<nv>::Type jointForce(Eigen::MatrixBase<D>& tau) const 
    { return tau.template segment<nv>(i_v); }
  };


  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */
  /* --- REVOLUTE X --------------------------------------------------------- */

  template<int axis> struct JointDataRevolute;
  template<int axis> struct JointModelRevolute;
  
  template<int axis>
  struct JointRevoluteHelper;

  template<>
  struct JointRevoluteHelper<0>
  {
    struct CartesianVector3
    {
      double w; 
      CartesianVector3(const double & w) : w(w) {}
      operator Eigen::Vector3d () { return Eigen::Vector3d(w,0,0); }
      friend Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3 & w2)
      { return Eigen::Vector3d(w1[0]+w2.w,w1[1],w1[2]); }
    };
  };

  template<int axis> 
  struct JointRevolute {
    struct BiasZero {};
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionRevolute 
    {
      MotionRevolute()                   : w(NAN) {}
      MotionRevolute( const double & w ) : w(w)  {}
      double w;

      operator Motion() const
      { 
	return Motion(Motion::Vector3::Zero(),typename JointRevoluteHelper<axis>::CartesianVector3(w));
      }
    }; // struct MotionRevolute

    friend const MotionRevolute& operator+ (const MotionRevolute& m, const BiasZero&) { return m; }
    friend Motion operator+( const MotionRevolute& m1, const Motion& m2)
    {
      return Motion( m2.linear(),m2.angular()+typename JointRevoluteHelper<axis>::CartesianVector3(m1.w)); 
    }    
    struct ConstraintRevolute
    { 
      template<typename D>
      MotionRevolute operator*( const Eigen::MatrixBase<D> & v ) const { return MotionRevolute(v[0]); }

      const ConstraintRevolute & transpose() const { return *this; }
     //template<typename D> D operator*( const Force& f ) const
     Force::Vector3::ConstFixedSegmentReturnType<1>::Type
     operator*( const Force& f ) const
     { return f.angular().head<1>(); }
    }; // struct ConstraintRevolute

    static Eigen::Matrix3d cartesianRotation(const double & angle); 
  };

  Motion operator^( const Motion& m1, const JointRevolute<0>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(w,0,0) = ( 0,zw,-yw )
     * nu1^(0,wx) = ( 0,vz1 wx,-vy1 wx,    0,wz1 wx,-wy1 wx)
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(0,v[2]*wx,-v[1]*wx),
		   Motion::Vector3(0,w[2]*wx,-w[1]*wx) );
  }

  Motion operator^( const Motion& m1, const JointRevolute<1>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,w,0) = ( -z,0,x )
     * nu1^(0,wx) = ( -vz1 wx,0,vx1 wx,    -wz1 wx,0,wx1 wx)
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(-v[2]*wx,0, v[0]*wx),
		   Motion::Vector3(-w[2]*wx,0, w[0]*wx) );
  }

  Motion operator^( const Motion& m1, const JointRevolute<2>::MotionRevolute& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,0,w) = ( y,-x,0 )
     * nu1^(0,wx) = ( vy1 wx,-vx1 wx,0,    wy1 wx,-wx1 wx,0 )
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion( Motion::Vector3(v[1]*wx,-v[0]*wx,0),
		   Motion::Vector3(w[1]*wx,-w[0]*wx,0) );
  }

  template<>
  Eigen::Matrix3d JointRevolute<0>::cartesianRotation(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	1,0,0,
	0,ca,-sa,
	0,sa,ca;
      return R3;
    }
  template<>
  Eigen::Matrix3d JointRevolute<1>::cartesianRotation(const double & angle)
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	ca, 0, sa,
	0 , 1,  0,
	-sa, 0,  ca;
      return R3;
    }
  template<>
  Eigen::Matrix3d JointRevolute<2>::cartesianRotation(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; sincos(angle,&sa,&ca);
      R3 << 
	ca,-sa,0,
	sa,ca,0,
	0,0,1;
      return R3;
    }



  template<int axis>
  struct traits< JointRevolute<axis> >
  {
    typedef JointDataRevolute<axis> JointData;
    typedef JointModelRevolute<axis> JointModel;
    typedef typename JointRevolute<axis>::ConstraintRevolute Constraint_t;
    typedef SE3 Transformation_t;
    typedef typename JointRevolute<axis>::MotionRevolute Motion_t;
    typedef typename JointRevolute<axis>::BiasZero Bias_t;
    enum {
      nq = 1,
      nv = 1
    };
  };

  template<int axis> struct traits< JointDataRevolute<axis> > { typedef JointRevolute<axis> Joint; };
  template<int axis> struct traits< JointModelRevolute<axis> > { typedef JointRevolute<axis> Joint; };

  template<int axis>
  struct JointDataRevolute : public JointDataBase< JointDataRevolute<axis> >
  {
    typedef JointRevolute<axis> Joint;
    SE3_JOINT_TYPEDEF;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    JointDataRevolute() : M(1)
    {
      M.translation(SE3::Vector3::Zero());
    }
  };

  template<int axis>
  struct JointModelRevolute : public JointModelBase< JointModelRevolute<axis> >
  {
    typedef JointRevolute<axis> Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevolute>::idx_q;
    using JointModelBase<JointModelRevolute>::idx_v;
    using JointModelBase<JointModelRevolute>::setIndexes;
    
    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.rotation(JointRevolute<axis>::cartesianRotation(q));
      data.v.w = v;
    }


  };


  typedef JointDataRevolute<0> JointDataRX;
  typedef JointModelRevolute<0> JointModelRX;

  typedef JointDataRevolute<1> JointDataRY;
  typedef JointModelRevolute<1> JointModelRY;

  typedef JointDataRevolute<2> JointDataRZ;
  typedef JointModelRevolute<2> JointModelRZ;

  // struct JointDataRX;
  // struct JointModelRX;

  // struct JointRX {
  //   struct BiasZero {};
  //   friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
  //   friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

  //   struct MotionRX 
  //   {
  //     MotionRX()                    : wx(NAN) {}
  //     MotionRX( const double & wx ) : wx(wx)  {}
  //     double wx;

  //     operator Motion() const
  //     { 
  // 	return Motion(Motion::Vector3::Zero(),Motion::Vector3(wx,0,0));
  //     }
  //   }; // struct MotionRX

  //   friend const MotionRX& operator+ (const MotionRX& m, const BiasZero&) { return m; }
  //   friend Motion operator+( const MotionRX& m1, const Motion& m2)
  //   {
  //     return Motion( m2.linear(),m2.angular()+Eigen::Vector3d::UnitX()*m1.wx); 
  //   }    
  //   friend Motion operator^( const Motion& m1, const MotionRX& m2)
  //   {
  //     /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
  //      * nu1^(0,w2) = ( v1^w2      , w1^w2 )
  //      * (x,y,z)^(0,0,w) = ( 0,zw,-yw )
  //      * nu1^(0,wx) = ( 0,vz1 wx,-vy1 wx,    0,wz1 wx,-wy1 wx)
  //      */
  //     const Motion::Vector3& v = m1.linear();
  //     const Motion::Vector3& w = m1.angular();
  //     const double & wx = m2.wx;
  //     return Motion( Motion::Vector3(0,v[2]*wx,-v[1]*wx),
  // 		     Motion::Vector3(0,w[2]*wx,-w[1]*wx) );
  //   }

  //   struct ConstraintRX
  //   { 
  //     template<typename D>
  //     MotionRX operator*( const Eigen::MatrixBase<D> & v ) const { return MotionRX(v[0]); }

  //     const ConstraintRX & transpose() const { return *this; }
  //    //template<typename D> D operator*( const Force& f ) const
  //    Force::Vector3::ConstFixedSegmentReturnType<1>::Type
  //    operator*( const Force& f ) const
  //    { return f.angular().head<1>(); }
  //   }; // struct ConstraintRX

  // };

  // template<>
  // struct traits<JointRX>
  // {
  //   typedef JointDataRX JointData;
  //   typedef JointModelRX JointModel;
  //   typedef typename JointRX::ConstraintRX Constraint_t;
  //   typedef SE3 Transformation_t;
  //   typedef JointRX::MotionRX Motion_t;
  //   typedef JointRX::BiasZero Bias_t;
  //   enum {
  //     nq = 1,
  //     nv = 1
  //   };
  // };

  // template<> struct traits<JointDataRX> { typedef JointRX Joint; };
  // template<> struct traits<JointModelRX> { typedef JointRX Joint; };

  // struct JointDataRX : public JointDataBase<JointDataRX>
  // {
  //   typedef JointRX Joint;
  //   SE3_JOINT_TYPEDEF;

  //   Constraint_t S;
  //   Transformation_t M;
  //   Motion_t v;
  //   Bias_t c;

  //   JointDataRX() : M(1)
  //   {
  //     M.translation(SE3::Vector3::Zero());
  //   }
  // };

  // struct JointModelRX : public JointModelBase<JointModelRX>
  // {
  //   typedef JointRX Joint;
  //   SE3_JOINT_TYPEDEF;

  //   using JointModelBase<JointModelRX>::idx_q;
  //   using JointModelBase<JointModelRX>::idx_v;
  //   using JointModelBase<JointModelRX>::setIndexes;
    
  //   JointData createData() const { return JointData(); }
  //   void calc( JointData& data, 
  // 	       const Eigen::VectorXd & qs, 
  // 	       const Eigen::VectorXd & vs ) const
  //   {
  //     const double & q = qs[idx_q()];
  //     const double & v = vs[idx_v()];

  //     data.M.rotation(rotationX(q));
  //     data.v.wx = v;
  //   }

  //   static inline Eigen::Matrix3d rotationX(const double & angle) 
  //   {
  //     Eigen::Matrix3d R3; 
  //     double ca,sa; sincos(angle,&sa,&ca);
  //     R3 << 
  // 	1,0,0,
  // 	0,ca,sa,
  // 	0,-sa,ca;
  //     return R3;
  //   }

  // };



  /* --- REVOLUTE FF -------------------------------------------------------- */
  /* --- REVOLUTE FF -------------------------------------------------------- */
  /* --- REVOLUTE FF -------------------------------------------------------- */


  struct JointDataFreeFlyer;
  struct JointModelFreeFlyer;

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
    typedef Motion Motion_t;
    typedef JointFreeFlyer::BiasZero Bias_t;
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
    Motion_t v;
    Bias_t c;

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
	       const Eigen::VectorXd & vs ) const
    {
      Eigen::VectorXd::ConstFixedSegmentReturnType<nq>::Type q = qs.segment<nq>(idx_q());
      data.v = vs.segment<nv>(idx_v());

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
