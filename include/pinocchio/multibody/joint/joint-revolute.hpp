#ifndef __se3_joint_revolute_hpp__
#define __se3_joint_revolute_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/sincos.hpp"

namespace se3
{

  template<int axis> struct JointDataRevolute;
  template<int axis> struct JointModelRevolute;
  
  namespace revolute
  {
    template<int axis>
    struct CartesianVector3
    {
      double w; 
      CartesianVector3(const double & w) : w(w) {}
      CartesianVector3() : w(1) {}
      operator Eigen::Vector3d (); // { return Eigen::Vector3d(w,0,0); }
    };
    template<>CartesianVector3<0>::operator Eigen::Vector3d () { return Eigen::Vector3d(w,0,0); }
    template<>CartesianVector3<1>::operator Eigen::Vector3d () { return Eigen::Vector3d(0,w,0); }
    template<>CartesianVector3<2>::operator Eigen::Vector3d () { return Eigen::Vector3d(0,0,w); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<0> & wx)
    { return Eigen::Vector3d(w1[0]+wx.w,w1[1],w1[2]); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<1> & wy)
    { return Eigen::Vector3d(w1[0],w1[1]+wy.w,w1[2]); }
    Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<2> & wz)
    { return Eigen::Vector3d(w1[0],w1[1],w1[2]+wz.w); }
  } // namespace revolute

  template<int axis> 
  struct JointRevolute {
    struct BiasZero 
    {
      operator Motion () const { return Motion::Zero(); }
    };
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionRevolute 
    {
      MotionRevolute()                   : w(NAN) {}
      MotionRevolute( const double & w ) : w(w)  {}
      double w;

      operator Motion() const
      { 
	return Motion(Motion::Vector3::Zero(),
		      (Motion::Vector3)typename revolute::CartesianVector3<axis>(w));
      }
    }; // struct MotionRevolute

    friend const MotionRevolute& operator+ (const MotionRevolute& m, const BiasZero&)
    { return m; }

    friend Motion operator+( const MotionRevolute& m1, const Motion& m2)
    {
      return Motion( m2.linear(),m2.angular()+typename revolute::CartesianVector3<axis>(m1.w)); 
    }    
    struct ConstraintRevolute
    { 
      template<typename D>
      MotionRevolute operator*( const Eigen::MatrixBase<D> & v ) const
      { return MotionRevolute(v[0]); }

      Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
      { 
	Eigen::Matrix<double,6,1> res;
	res.head<3>() = m.translation().cross( m.rotation().col(axis));
	res.tail<3>() = m.rotation().col(axis);
	return res;
      }

      struct TransposeConst
      {
	const ConstraintRevolute & ref; 
	TransposeConst(const ConstraintRevolute & ref) : ref(ref) {} 

	Force::Vector3::ConstFixedSegmentReturnType<1>::Type
	operator*( const Force& f ) const
	{ return f.angular().segment<1>(axis); }

	/* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
	template<typename D>
	friend typename Eigen::MatrixBase<D>::ConstRowXpr
	operator*( const TransposeConst &, const Eigen::MatrixBase<D> & F )
	{
	  assert(F.rows()==6);
	  return F.row(3+axis);
	}

      };
      TransposeConst transpose() const { return TransposeConst(*this); }


    /* CRBA joint operators
     *   - ForceSet::Block = ForceSet
     *   - ForceSet operator* (Inertia Y,Constraint S)
     *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
     *   - SE3::act(ForceSet::Block)
     */
      operator ConstraintXd () const
      {
	Eigen::Matrix<double,6,1> S;
	S << Eigen::Vector3d::Zero(), (Eigen::Vector3d)revolute::CartesianVector3<axis>();
	return ConstraintXd(S);
      }
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
      double ca,sa; SINCOS (angle,&sa,&ca);
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
      double ca,sa; SINCOS (angle,&sa,&ca);
      R3 << 
	 ca, 0,  sa,
	  0, 1,   0,
	-sa, 0,  ca;
      return R3;
    }
  template<>
  Eigen::Matrix3d JointRevolute<2>::cartesianRotation(const double & angle) 
    {
      Eigen::Matrix3d R3; 
      double ca,sa; SINCOS (angle,&sa,&ca);
      R3 << 
	ca,-sa,0,
	sa,ca,0,
	0,0,1;
      return R3;
    }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const JointRevolute<0>::ConstraintRevolute & )
  { 
    /* Y(:,3) = ( 0,-z, y,  I00+yy+zz,  I01-xy   ,  I02-xz   ) */
    const double 
      &m = Y.mass(),
      &x = Y.lever()[0],
      &y = Y.lever()[1],
      &z = Y.lever()[2];
    const Inertia::Symmetric3 & I = Y.inertia();
    Eigen::Matrix<double,6,1> res; res << 0.0,-m*z,m*y,
				     I(0,0)+m*(y*y+z*z),
    				     I(0,1)-m*x*y,
    				     I(0,2)-m*x*z ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const JointRevolute<1>::ConstraintRevolute & )
  { 
    /* Y(:,4) = ( z, 0,-x,  I10-xy   ,  I11+xx+zz,  I12-yz   ) */
    const double 
      &m = Y.mass(),
      &x = Y.lever()[0],
      &y = Y.lever()[1],
      &z = Y.lever()[2];
    const Inertia::Symmetric3 & I = Y.inertia();
    Eigen::Matrix<double,6,1> res; res << m*z,0,-m*x,
				     I(1,0)-m*x*y,
    				     I(1,1)+m*(x*x+z*z),
				     I(1,2)-m*y*z ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const JointRevolute<2>::ConstraintRevolute & )
  { 
    /* Y(:,5) = (-y, x, 0,  I20-xz   ,  I21-yz   ,  I22+xx+yy) */
    const double 
      &m = Y.mass(),
      &x = Y.lever()[0],
      &y = Y.lever()[1],
      &z = Y.lever()[2];
    const Inertia::Symmetric3 & I = Y.inertia();
    Eigen::Matrix<double,6,1> res; res << -m*y,m*x,0,
				     I(2,0)-m*x*z,
				     I(2,1)-m*y*z,
				     I(2,2)+m*(x*x+y*y) ;
    return res;
  }

  namespace internal 
  {
    // TODO: I am not able to write the next three lines as a template. Why?
    template<>
    struct ActionReturn<typename JointRevolute<0>::ConstraintRevolute >  
    { typedef Eigen::Matrix<double,6,1> Type; };
    template<>
    struct ActionReturn<typename JointRevolute<1>::ConstraintRevolute >  
    { typedef Eigen::Matrix<double,6,1> Type; };
    template<>
    struct ActionReturn<typename JointRevolute<2>::ConstraintRevolute >  
    { typedef Eigen::Matrix<double,6,1> Type; };
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
    typedef Eigen::Matrix<double,6,1> F_t;
    enum {
      NQ = 1,
      NV = 1
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

    F_t F;

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
	       const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      data.M.rotation(JointRevolute<axis>::cartesianRotation(q));
    }

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

  typedef JointRevolute<0> JointRX;
  typedef JointDataRevolute<0> JointDataRX;
  typedef JointModelRevolute<0> JointModelRX;

  typedef JointRevolute<1> JointRY;
  typedef JointDataRevolute<1> JointDataRY;
  typedef JointModelRevolute<1> JointModelRY;

  typedef JointRevolute<2> JointRZ;
  typedef JointDataRevolute<2> JointDataRZ;
  typedef JointModelRevolute<2> JointModelRZ;

} //namespace se3

#endif // ifndef __se3_joint_revolute_hpp__
