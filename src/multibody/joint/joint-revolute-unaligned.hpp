#ifndef __se3_joint_revolute_unaligned_hpp__
#define __se3_joint_revolute_unaligned_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/math/sincos.hpp"

namespace se3
{

  struct JointDataRevoluteUnaligned;
  struct JointModelRevoluteUnaligned;
  
  struct JointRevoluteUnaligned {
    struct BiasZero 
    {
      operator Motion () const { return Motion::Zero(); }
    };
    friend const Motion & operator+ ( const Motion& v, const BiasZero&) { return v; }
    friend const Motion & operator+ ( const BiasZero&,const Motion& v) { return v; }

    struct MotionRevoluteUnaligned 
    {
      // Empty constructor needed for now to avoid compilation errors
      MotionRevoluteUnaligned() : axis(Motion::Vector3::Constant(NAN)), w(NAN) {} 
      MotionRevoluteUnaligned( const Motion::Vector3 & axis, const double & w ) : axis(axis), w(w)  {}

      operator Motion() const
      { 
	return Motion(Motion::Vector3::Zero(),
		      (Motion::Vector3)(axis*w));
      }

      Motion::Vector3 axis; 
      double w;
    }; // struct MotionRevoluteUnaligned

    friend const MotionRevoluteUnaligned& operator+ (const MotionRevoluteUnaligned& m, const BiasZero&)
    { return m; }

    friend Motion operator+( const MotionRevoluteUnaligned& m1, const Motion& m2)
    {
      return Motion( m2.linear(), m1.w*m1.axis+m2.angular() );
    }

    struct ConstraintRevoluteUnaligned
    { 
      // Empty constructor needed to avoid compilation error for now.
      ConstraintRevoluteUnaligned() : axis(Eigen::Vector3d::Constant(NAN)) {}
      ConstraintRevoluteUnaligned(const Motion::Vector3 & _axis) : axis(_axis) {}
      
      template<typename D> //todo : check operator is good
      MotionRevoluteUnaligned operator*( const Eigen::MatrixBase<D> & v ) const
      { 
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(D,1);
	return MotionRevoluteUnaligned(axis,v[0]); 
      }

      Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
      { 
	/* X*S = [ R pxR ; 0 R ] [ 0 ; a ] = [ px(Ra) ; Ra ] */
	Eigen::Matrix<double,6,1> res;
	const Eigen::Vector3d & Rx = m.rotation()* axis;
	res.head<3>() = m.translation().cross(Rx);
	res.tail<3>() = Rx;
	return res;
      }

      struct TransposeConst
      {
	const ConstraintRevoluteUnaligned & ref; 
	TransposeConst(const ConstraintRevoluteUnaligned & ref) : ref(ref) {} 

	const Eigen::Matrix<double, 1, 1>
	operator*( const Force& f ) const
	{
	  return ref.axis.transpose()*f.angular();
	}

	/* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
	template<typename D>
	friend typename Eigen::Matrix<double,1,D::ColsAtCompileTime>
	operator*( const TransposeConst & tc, const Eigen::MatrixBase<D> & F )
	{
	  /* Return ax.T * F[3:end,:] */
	  assert(F.rows()==6); //TODO 
	  Motion::Vector3 ax(tc.ref.axis);
	  return Eigen::Matrix<double,1,D::ColsAtCompileTime>
	    (ax.transpose()*F.template bottomRows<3>());
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
	S << Eigen::Vector3d::Zero(), axis;
	return ConstraintXd(S);
      }
      Motion::Vector3 axis; 
    }; // struct ConstraintRevoluteUnaligned

  }; // struct JoinRevoluteUnaligned

  Motion operator^( const Motion& m1, const JointRevoluteUnaligned::MotionRevoluteUnaligned & m2)
  {
    /* m1xm2 = [ v1xw2 + w1xv2; w1xw2 ] = [ v1xw2; w1xw2 ] */
    const Motion::Vector3& v1 = m1.linear();
    const Motion::Vector3& w1 = m1.angular();
    const Motion::Vector3& w2 = m2.axis * m2.w ;
    return Motion( v1.cross(w2),w1.cross(w2));
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const JointRevoluteUnaligned::ConstraintRevoluteUnaligned & cru)
  { 
    /* YS = [ m mcx ; -mcx I-mcxcx ] [ 0 ; w ] = [ mcxw ; Iw -mcxcxw ] */
    const double &m                 = Y.mass();
    const Inertia::Vector3 & c      = Y.lever();
    const Inertia::Symmetric3 & I   = Y.inertia();

    const Motion::Vector3 mcxw = m*c.cross(cru.axis);
    Eigen::Matrix<double,6,1> res;
    res.head<3>() = mcxw;
    res.tail<3>() = I*cru.axis - c.cross(mcxw);
    return res;
  }
  namespace internal 
  {
    template<>
    struct ActionReturn<JointRevoluteUnaligned::ConstraintRevoluteUnaligned >  
    { typedef Eigen::Matrix<double,6,1> Type; };
  }


  template<>
  struct traits< JointRevoluteUnaligned >
  {
    typedef JointDataRevoluteUnaligned JointData;
    typedef JointModelRevoluteUnaligned JointModel;
    typedef typename JointRevoluteUnaligned::ConstraintRevoluteUnaligned Constraint_t;
    typedef SE3 Transformation_t;
    typedef typename JointRevoluteUnaligned::MotionRevoluteUnaligned Motion_t;
    typedef typename JointRevoluteUnaligned::BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,1> F_t;
    enum {
      NQ = 1,
      NV = 1
    };
  };

  template<> struct traits<JointDataRevoluteUnaligned> { typedef JointRevoluteUnaligned Joint; };
  template<> struct traits<JointModelRevoluteUnaligned> { typedef JointRevoluteUnaligned Joint; };

  struct JointDataRevoluteUnaligned : public JointDataBase< JointDataRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned Joint;
    SE3_JOINT_TYPEDEF;

    Transformation_t M;
    Constraint_t S;
    Motion_t v;
    Bias_t c;

    F_t F;
    /* It should not be necessary to copy axis in jdata, however a current bug
     * in the fusion visitor prevents a proper access to jmodel::axis. A
     * by-pass is to access to a copy of it in jdata. */
    Eigen::Vector3d axis;
    Eigen::AngleAxisd angleaxis;

    /* The empty constructor is needed for the variant. */
    JointDataRevoluteUnaligned() 
      : M(1),S(Eigen::Vector3d::Constant(NAN)),v(Eigen::Vector3d::Constant(NAN),NAN)
      , axis(Eigen::Vector3d::Constant(NAN))
      , angleaxis( NAN,Eigen::Vector3d::Constant(NAN))
    {}
    JointDataRevoluteUnaligned(const Motion::Vector3 & axis) 
      : M(1),S(axis),v(axis,NAN)
      ,axis(axis)
      ,angleaxis(NAN,axis)
    {}
  };

  struct JointModelRevoluteUnaligned : public JointModelBase< JointModelRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevoluteUnaligned>::idx_q;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_v;
    using JointModelBase<JointModelRevoluteUnaligned>::setIndexes;
    
    /* The empty constructor is needed for the variant. */
    JointModelRevoluteUnaligned() : axis(Eigen::Vector3d::Constant(NAN))   {}
    JointModelRevoluteUnaligned( const Motion::Vector3 & axis ) : axis(axis)
    {
      assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointData createData() const { return JointData(axis); }
    void calc( JointData& data, 
	       const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];

      /* It should not be necessary to copy axis in jdata, however a current bug
       * in the fusion visitor prevents a proper access to jmodel::axis. A
       * by-pass is to access to a copy of it in jdata. */
      data.angleaxis.angle() = q;
      data.M.rotation(data.angleaxis.toRotationMatrix());
    }

    void calc( JointData& data, 
	       const Eigen::VectorXd & qs, 
	       const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      /* It should not be necessary to copy axis in jdata, however a current bug
       * in the fusion visitor prevents a proper access to jmodel::axis. A
       * by-pass is to access to a copy of it in jdata. */
      data.angleaxis.angle() = q;
      data.M.rotation(data.angleaxis.toRotationMatrix());
      data.v.w = v;
    }

    //protected: TODO
    Eigen::Vector3d axis;
  };

} //namespace se3


#endif // ifndef __se3_joint_revolute_unaligned_hpp__
