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

#ifndef __se3_joint_revolute_hpp__
#define __se3_joint_revolute_hpp__

// #include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"

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

  template<int axis> struct MotionRevolute;

  template<int axis>
  struct traits< MotionRevolute < axis > >
  {
    typedef double Scalar_t;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
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
  }; // traits MotionRevolute


  
  template<int axis>
  struct MotionRevolute : MotionBase < MotionRevolute <axis > >
  {
    SPATIAL_TYPEDEF_TEMPLATE(MotionRevolute);

    MotionRevolute()                   : w(NAN) {}
    MotionRevolute( const double & w ) : w(w)  {}
    double w;

    operator Motion() const
    {
      return Motion(  Motion::Vector3::Zero(),
        (Vector3)typename revolute::CartesianVector3<axis>(w)
        );
    }
  }; // struct MotionRevolute

  template <int axis >
  const MotionRevolute<axis>& operator+ (const MotionRevolute<axis>& m, const BiasZero&)
  { return m; }

  template<int axis >
  Motion operator+( const MotionRevolute<axis>& m1, const Motion& m2)
  {
    return Motion( m2.linear(),m2.angular()+typename revolute::CartesianVector3<axis>(m1.w)); 
  }

  template<int axis> struct ConstraintRevolute;

  template<int axis>
  struct traits< ConstraintRevolute<axis> >
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

  template<int axis>
  struct ConstraintRevolute : ConstraintBase < ConstraintRevolute <axis > >
  { 
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRevolute);
    enum { NV = 1, Options = 0 };
    typedef typename traits<ConstraintRevolute>::JointMotion JointMotion;
    typedef typename traits<ConstraintRevolute>::JointForce JointForce;
    typedef typename traits<ConstraintRevolute>::DenseBase DenseBase;

    template<typename D>
    MotionRevolute<axis> operator*( const Eigen::MatrixBase<D> & v ) const
    { return MotionRevolute<axis>(v[0]); }

    Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
    { 
      Eigen::Matrix<double,6,1> res;
      res.head<3>() = m.translation().cross( m.rotation().col(axis));
      res.tail<3>() = m.rotation().col(axis);
      return res;
    }

    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      const ConstraintRevolute<axis> & ref; 
      TransposeConst(const ConstraintRevolute<axis> & ref) : ref(ref) {} 

      typename Force::Vector3::template ConstFixedSegmentReturnType<1>::Type
      operator*( const Force& f ) const
      { return f.angular().template segment<1>(axis); }

        /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename D>
      friend typename Eigen::MatrixBase<D>::ConstRowXpr
      operator*( const TransposeConst &, const Eigen::MatrixBase<D> & F )
      {
        assert(F.rows()==6);
        return F.row(Inertia::ANGULAR + axis);
      }
    }; // struct TransposeConst

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

  template<int axis> 
  struct JointRevolute {
      static Eigen::Matrix3d cartesianRotation(const double & angle); 
  };

  Motion operator^( const Motion& m1, const MotionRevolute<0>& m2)
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
                   Motion::Vector3(0,w[2]*wx,-w[1]*wx)
                 );
  }

  Motion operator^( const Motion& m1, const MotionRevolute<1>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,w,0) = ( -z,0,x )
     * nu1^(0,wx) = ( -vz1 wx,0,vx1 wx,    -wz1 wx,0,wx1 wx)
     */
    const Motion::Vector3& v = m1.linear();
    const Motion::Vector3& w = m1.angular();
    const double & wx = m2.w;
    return Motion(  Motion::Vector3(-v[2]*wx,0, v[0]*wx),
                    Motion::Vector3(-w[2]*wx,0, w[0]*wx)
                 );
  }

  Motion operator^( const Motion& m1, const MotionRevolute<2>& m2)
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
                   Motion::Vector3(w[1]*wx,-w[0]*wx,0)
                 );
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
  operator*( const Inertia& Y,const ConstraintRevolute<0> & )
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
  operator*( const Inertia& Y,const ConstraintRevolute<1> & )
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
  operator*( const Inertia& Y,const ConstraintRevolute<2> & )
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
    template<int axis>
    struct ActionReturn<ConstraintRevolute<axis> >
    { typedef Eigen::Matrix<double,6,1> Type; };
  }



  template<int axis>
  struct traits< JointRevolute<axis> >
  {
    typedef JointDataRevolute<axis> JointData;
    typedef JointModelRevolute<axis> JointModel;
    typedef ConstraintRevolute<axis> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionRevolute<axis> Motion_t;
    typedef BiasZero Bias_t;
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
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;

    JointDataRevolute() : M(1)
    {
      M.translation(SE3::Vector3::Zero());
    }

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }
  }; // struct JointDataRevolute

  template<int axis>
  struct JointModelRevolute : public JointModelBase< JointModelRevolute<axis> >
  {
    typedef JointRevolute<axis> Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelRevolute>::id;
    using JointModelBase<JointModelRevolute>::idx_q;
    using JointModelBase<JointModelRevolute>::idx_v;
    using JointModelBase<JointModelRevolute>::lowerPosLimit;
    using JointModelBase<JointModelRevolute>::upperPosLimit;
    using JointModelBase<JointModelRevolute>::maxEffortLimit;
    using JointModelBase<JointModelRevolute>::maxVelocityLimit;
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

    bool operator == (const JointModelRevolute<axis>& /*Other*/) const
    {
      return true; // TODO ??
    }

  }; // struct JointModelRevolute

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
