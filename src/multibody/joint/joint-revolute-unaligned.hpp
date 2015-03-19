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
      MotionRevoluteUnaligned() : axis(Motion::Vector3::Zero()), w(NAN) {} //will lead to error computations
      MotionRevoluteUnaligned( const Motion::Vector3 & axis, const double & w ) : axis(axis), w(w)  {}
      Motion::Vector3 axis; 
      double w;

      operator Motion() const
      { 
  return Motion(Motion::Vector3::Zero(),
          (Motion::Vector3)(axis)*w);
      }
    }; // struct MotionRevoluteUnaligned

    friend const MotionRevoluteUnaligned& operator+ (const MotionRevoluteUnaligned& m, const BiasZero&)
    { return m; }

    friend Motion operator+( const MotionRevoluteUnaligned& m1, const Motion& m2)
    {
      return Motion( m2.linear(),m2.angular()+ m1.w*m1.axis);
    }

    struct ConstraintRevoluteUnaligned
    { 
      Motion::Vector3 axis; 
      ConstraintRevoluteUnaligned(const Motion::Vector3 & axis) : axis(axis) {}
      
      template<typename D> //todo : check operator is good
      MotionRevoluteUnaligned operator*( const Eigen::MatrixBase<D> & v ) const
      { return MotionRevoluteUnaligned(axis,v[0]); }

      Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
      { 
  Eigen::Matrix<double,6,1> res;
  res.head<3>() = m.translation().cross(m.rotation()*axis);
  res.tail<3>() = m.rotation()* axis;
  return res;
      }

      struct TransposeConst
      {
  const ConstraintRevoluteUnaligned & ref; 
  TransposeConst(const ConstraintRevoluteUnaligned & ref) : ref(ref) {} 

  // Force::Vector3::ConstFixedSegmentReturnType<1>::Type
  const Eigen::Matrix<double, 1, 1> //TODO typedef this line to be clean
  operator*( const Force& f ) const
  {
    return ref.axis.transpose()*f.angular();
  }

   /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
  template<typename D>
  //friend typename Eigen::MatrixBase<D>::ConstRowXpr
  friend typename Eigen::Matrix<double,1,Eigen::Dynamic> //SHould be F::NbCols
  operator*( const TransposeConst & tc, const Eigen::MatrixBase<D> & F )
  {
    Motion::Vector3 ax(tc.ref.axis);
    Eigen::MatrixXd rr(Eigen::MatrixXd::Random(1,6));
    Eigen::MatrixXd bl(Eigen::MatrixXd::Random(3,6));
    Eigen::MatrixXd bl2(F.bottomRows(3));
    //return ax.transpose()*bl2;
    return ax.transpose()*F.bottomRows(3);
    assert(F.rows()==6); //TODO 
    //return F.row(4); //<-- compile but wrong result of course
            // ax.transpose()*F.bottomRows<3>();
    //ref.axis.transpose() * F.bottomRows<3>());
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
    }; // struct ConstraintRevoluteUnaligned

  }; // struct JoinRevoluteUnaligned

  Motion operator^( const Motion& m1, const JointRevoluteUnaligned::MotionRevoluteUnaligned & m2)
  {
    
    const Motion::Vector3& v1 = m1.linear();
    const Motion::Vector3& w1 = m1.angular();
    //const Motion::Vector3& w2 = m2.Motion().m_w;
    const Motion::Vector3& w2 = m2.axis * m2.w ;
    return Motion( v1.cross(w2),w1.cross(w2));
  }



  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const JointRevoluteUnaligned::ConstraintRevoluteUnaligned & cru)
  { 
    const double &m                 = Y.mass();
    const Inertia::Vector3 & c      = Y.lever();
    const Inertia::Symmetric3 & I   = Y.inertia();

    const Motion::Vector3 cxw = c.cross(cru.axis);
    Eigen::Matrix<double,6,1> res;
    res.head<3>() = cxw;
    res.tail<3>() = I*cru.axis - m*c.cross(cxw);
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

    JointDataRevoluteUnaligned(const Motion::Vector3 & axis) : M(1),S(axis)//() : M(1),S(Motion::Vector3::Zero())
    {    }
    // JointDataRevoluteUnaligned(const Motion::Vector3 & axis) : M(1),S(axis)
    // {    }
  };

  struct JointModelRevoluteUnaligned : public JointModelBase< JointModelRevoluteUnaligned >
  {
    typedef JointRevoluteUnaligned Joint;
    SE3_JOINT_TYPEDEF;

    using JointModelBase<JointModelRevoluteUnaligned>::idx_q;
    using JointModelBase<JointModelRevoluteUnaligned>::idx_v;
    using JointModelBase<JointModelRevoluteUnaligned>::setIndexes;
    
    JointModelRevoluteUnaligned( const Motion::Vector3 & axis ) : axis(axis)
    {
     // TODO ASSERT axis is unit
      //assert(axis.isUnitary() && "Rotation axis is not unitary");
    }

    JointData createData() const { return JointData(axis); }
    void calc( JointData& data, 
         const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      data.M.rotation(Eigen::AngleAxisd(q,axis).matrix());
    }

    void calc( JointData& data, 
         const Eigen::VectorXd & qs, 
         const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.rotation(Eigen::AngleAxisd(q,axis).matrix());
      data.v.w = v;
    }

    protected:
      Motion::Vector3 axis;

  };

  typedef JointRevoluteUnaligned JointRU;
  typedef JointDataRevoluteUnaligned JointDataRU;
  typedef JointModelRevoluteUnaligned JointModelRU;

} //namespace se3


#endif // ifndef __se3_joint_revolute_unaligned_hpp__