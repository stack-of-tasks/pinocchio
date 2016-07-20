//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_joint_revolute_unbounded_hpp__
#define __se3_joint_revolute_unbounded_hpp__

#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-dense.hpp"

#include <stdexcept>

namespace se3
{

  template<int axis> struct JointDataRevoluteUnbounded;
  template<int axis> struct JointModelRevoluteUnbounded;
  
  template<int axis> struct SE3RevoluteUnbounded;
  template<int axis> struct MotionRevoluteUnbounded;

  namespace revoluteunbounded
  {
    template<int axis>
    struct CartesianVector3
    {
      double w; 
      CartesianVector3(const double w) : w(w) {}
      CartesianVector3() : w(NAN) {}
      
      Eigen::Vector3d vector() const;
      operator Eigen::Vector3d () const { return vector(); }
    }; // struct CartesianVector3
    template<> inline Eigen::Vector3d CartesianVector3<0>::vector() const { return Eigen::Vector3d(w,0,0); }
    template<> inline Eigen::Vector3d CartesianVector3<1>::vector() const { return Eigen::Vector3d(0,w,0); }
    template<> inline Eigen::Vector3d CartesianVector3<2>::vector() const { return Eigen::Vector3d(0,0,w); }
    
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<0> & wx)
    { return Eigen::Vector3d(w1[0]+wx.w,w1[1],w1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<1> & wy)
    { return Eigen::Vector3d(w1[0],w1[1]+wy.w,w1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & w1,const CartesianVector3<2> & wz)
    { return Eigen::Vector3d(w1[0],w1[1],w1[2]+wz.w); }
  } // namespace revoluteunbounded


  template<int axis>
  struct traits< MotionRevoluteUnbounded < axis > >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Vector3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
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
  }; // traits MotionRevoluteUnbounded

  template<int axis>
  struct MotionRevoluteUnbounded : MotionBase < MotionRevoluteUnbounded <axis > >
  {
    SPATIAL_TYPEDEF_TEMPLATE(MotionRevoluteUnbounded);

    MotionRevoluteUnbounded()                   : w(NAN) {}
    MotionRevoluteUnbounded( const double & w ) : w(w)  {}
    double w;

    operator Motion() const
    {
      return Motion(Motion::Vector3::Zero(),
        typename revoluteunbounded::CartesianVector3<axis>(w).vector()
        );
    }
  }; // struct MotionRevoluteUnbounded

  template <int axis >
  const MotionRevoluteUnbounded<axis>& operator+ (const MotionRevoluteUnbounded<axis>& m, const BiasZero&)
  { return m; }

  template<int axis >
  Motion operator+( const MotionRevoluteUnbounded<axis>& m1, const Motion& m2)
  {
    return Motion( m2.linear(),m2.angular()+typename revoluteunbounded::CartesianVector3<axis>(m1.w)); 
  }

  template<int axis> struct ConstraintRevoluteUnbounded;

  template<int axis>
  struct traits< ConstraintRevoluteUnbounded<axis> >
  {
    typedef double Scalar;
    typedef Eigen::Matrix<double,3,1,0> Vector3;
    typedef Eigen::Matrix<double,4,1,0> Vector4;
    typedef Eigen::Matrix<double,6,1,0> Vector6;
    typedef Eigen::Matrix<double,3,3,0> Matrix3;
    typedef Eigen::Matrix<double,4,4,0> Matrix4;
    typedef Eigen::Matrix<double,6,6,0> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
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
    typedef Eigen::Matrix<Scalar,1,1,0> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,0> JointForce;
    typedef Eigen::Matrix<Scalar,6,1> DenseBase;
  }; // traits ConstraintRevoluteUnbounded

  template<int axis>
  struct ConstraintRevoluteUnbounded : ConstraintBase < ConstraintRevoluteUnbounded <axis > >
  { 
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRevoluteUnbounded);
    enum { NV = 1, Options = 0 };
    typedef typename traits<ConstraintRevoluteUnbounded>::JointMotion JointMotion;
    typedef typename traits<ConstraintRevoluteUnbounded>::JointForce JointForce;
    typedef typename traits<ConstraintRevoluteUnbounded>::DenseBase DenseBase;

    template<typename D>
    MotionRevoluteUnbounded<axis> operator*( const Eigen::MatrixBase<D> & v ) const
    { return MotionRevoluteUnbounded<axis>(v[0]); }

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
      const ConstraintRevoluteUnbounded<axis> & ref; 
      TransposeConst(const ConstraintRevoluteUnbounded<axis> & ref) : ref(ref) {} 

      typename Force::ConstAngular_t::template ConstFixedSegmentReturnType<1>::Type
      operator* (const Force & f) const
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
      S << Eigen::Vector3d::Zero(), revoluteunbounded::CartesianVector3<axis>(1).vector();
      return ConstraintXd(S);
    }
  }; // struct ConstraintRevoluteUnbounded

  template<int axis> 
  struct JointRevoluteUnbounded {
      static Eigen::Matrix3d cartesianRotation(const double & ca, const double & sa); 
  };

  inline Motion operator^( const Motion& m1, const MotionRevoluteUnbounded<0>& m2)
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

  inline Motion operator^( const Motion& m1, const MotionRevoluteUnbounded<1>& m2)
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

  inline Motion operator^( const Motion& m1, const MotionRevoluteUnbounded<2>& m2)
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

  template<> inline
  Eigen::Matrix3d JointRevoluteUnbounded<0>::cartesianRotation(const double & ca, const double & sa) 
  {
    Eigen::Matrix3d R3; 
    R3 << 
    1,0,0,
    0,ca,-sa,
    0,sa,ca;
    return R3;
  }

  template<> inline
  Eigen::Matrix3d JointRevoluteUnbounded<1>::cartesianRotation(const double & ca, const double & sa)
  {
    Eigen::Matrix3d R3; 
    R3 << 
    ca, 0,  sa,
    0, 1,   0,
    -sa, 0,  ca;
    return R3;
  }

  template<> inline
  Eigen::Matrix3d JointRevoluteUnbounded<2>::cartesianRotation(const double & ca, const double & sa) 
  {
    Eigen::Matrix3d R3; 
    R3 << 
    ca,-sa,0,
    sa,ca,0,
    0,0,1;
    return R3;
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  Eigen::Matrix<double,6,1> inline
  operator*( const Inertia& Y,const ConstraintRevoluteUnbounded<0> & )
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
  Eigen::Matrix<double,6,1> inline
  operator*( const Inertia& Y,const ConstraintRevoluteUnbounded<1> & )
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
  Eigen::Matrix<double,6,1> inline
  operator*( const Inertia& Y,const ConstraintRevoluteUnbounded<2> & )
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
  
  /* [ABA] I*S operator (Inertia Y,Constraint S) */
  template<int axis>
  inline const Eigen::MatrixBase<const Inertia::Matrix6>::ColXpr
  operator*(const Inertia::Matrix6 & Y,const ConstraintRevoluteUnbounded<axis> & )
  {
    return Y.col(Inertia::ANGULAR + axis);
  }
  
  template<int axis>
  inline Eigen::MatrixBase<Inertia::Matrix6>::ColXpr
  operator*(Inertia::Matrix6 & Y,const ConstraintRevoluteUnbounded<axis> & )
  {
    return Y.col(Inertia::ANGULAR + axis);
  }

  namespace internal 
  {
    template<int axis>
    struct ActionReturn<ConstraintRevoluteUnbounded<axis> >
    { typedef Eigen::Matrix<double,6,1> Type; };
  }



  template<int axis>
  struct traits< JointRevoluteUnbounded<axis> >
  {
    enum {
      NQ = 2,
      NV = 1
    };
    
    typedef JointDataRevoluteUnbounded<axis> JointDataDerived;
    typedef JointModelRevoluteUnbounded<axis> JointModelDerived;
    typedef ConstraintRevoluteUnbounded<axis> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionRevoluteUnbounded<axis> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };

  template<int axis> struct traits< JointDataRevoluteUnbounded<axis> > { typedef JointRevoluteUnbounded<axis> JointDerived; };
  template<int axis> struct traits< JointModelRevoluteUnbounded<axis> > { typedef JointRevoluteUnbounded<axis> JointDerived; };

  template<int axis>
  struct JointDataRevoluteUnbounded : public JointDataBase< JointDataRevoluteUnbounded<axis> >
  {
    typedef JointRevoluteUnbounded<axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;
    F_t F;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnbounded() : M(1), U(), Dinv(), UDinv()
    {}

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F, U, Dinv, UDinv);
    }
  }; // struct JointDataRevoluteUnbounded

  template<int axis>
  struct JointModelRevoluteUnbounded : public JointModelBase< JointModelRevoluteUnbounded<axis> >
  {
    typedef JointRevoluteUnbounded<axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelRevoluteUnbounded>::id;
    using JointModelBase<JointModelRevoluteUnbounded>::idx_q;
    using JointModelBase<JointModelRevoluteUnbounded>::idx_v;
    using JointModelBase<JointModelRevoluteUnbounded>::setIndexes;
    typedef Motion::Vector3 Vector3;
    typedef double Scalar;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    void calc( JointDataDerived& data, 
     const Eigen::VectorXd & qs ) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());

      const double & ca = q(0);
      const double & sa = q(1);

      data.M.rotation(JointRevoluteUnbounded<axis>::cartesianRotation(ca,sa));
    }

    void calc( JointDataDerived& data, 
     const Eigen::VectorXd & qs, 
     const Eigen::VectorXd & vs ) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      const double & ca = q(0);
      const double & sa = q(1);
      const double & v = q_dot(0);

      data.M.rotation(JointRevoluteUnbounded<axis>::cartesianRotation(ca,sa));
      data.v.w = v;
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = 1./I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
      data.UDinv = data.U * data.Dinv[0];
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }


    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & q = qs.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NV>::Type & q_dot = vs.segment<NV> (idx_v ());

      const double & ca = q(0);
      const double & sa = q(1);
      const double & omega = q_dot(0);

      double cosOmega,sinOmega; SINCOS (omega, &sinOmega, &cosOmega);
      double norm2p = q.squaredNorm();

      ConfigVector_t result;
      result << (1.5-.5*norm2p) * (cosOmega * ca - sinOmega * sa),
                (1.5-.5*norm2p) * (sinOmega * ca + cosOmega * sa);
      return result; 
    }


    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    { 
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qi = q0.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qf = q1.segment<NQ> (idx_q ());

      const double & c0 = qi(0), s0 = qi(1);
      const double & c1 = qf(0), s1 = qf(1);

      assert ( (qi.norm() - 1) < 1e-8 && "initial configuration not normalized");
      assert ( (qf.norm() - 1) < 1e-8 && "final configuration not normalized");
      double cosTheta = c0*c1 + s0*s1;
      double sinTheta = c0*s1 - s0*c1;
      double theta = atan2(sinTheta, cosTheta);
      assert (fabs (sin (theta) - sinTheta) < 1e-8);

      ConfigVector_t result;
      if (fabs (theta) > 1e-6 && fabs (theta) < PI - 1e-6)
      {
        result = (sin ((1-u)*theta)/sinTheta) * qi 
                  + (sin (u*theta)/sinTheta) * qf;
      } 
      else if (fabs (theta) < 1e-6) // theta = 0
      {
        result = (1-u) * qi + u * qf;
      }
      else // theta = +-PI
      {
        double theta0 = atan2 (s0, c0);
        result << cos (theta0 + u * theta),
                  sin (theta0 + u * theta);
      }
      
      return result; 
    }

    ConfigVector_t random_impl() const
    { 
      ConfigVector_t result(ConfigVector_t::Random());
      return result;
    } 


    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & , const ConfigVector_t &  ) const throw (std::runtime_error)
    { 
      ConfigVector_t result;
      double angle = -PI + 2*PI * rand()/RAND_MAX;
      double ca,sa; SINCOS (angle, &sa, &ca);

      result << ca, sa;
      return result;
    } 


    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qi = q0.segment<NQ> (idx_q ());
      typename Eigen::VectorXd::ConstFixedSegmentReturnType<NQ>::Type & qf = q1.segment<NQ> (idx_q ());

      const double & c0 = qi(0), s0 = qi(1);
      const double & c1 = qf(0), s1 = qf(1);

      TangentVector_t result;
      result << atan2 (s0*c1 - s1*c0, c0*c1 + s0*s1);
      return result; 
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return difference_impl(q0,q1).norm();
    }

    ConfigVector_t neutralConfiguration_impl() const
    { 
      ConfigVector_t q;
      q << 1,0;
      return q;
    } 

    JointModelDense<NQ, NV> toDense_impl() const
    {
      return JointModelDense<NQ, NV>( id(),
                                      idx_q(),
                                      idx_v()
                                    );
    }

    static const std::string shortname();

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelRevoluteUnbounded <axis> > & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v();
    }

  }; // struct JointModelRevoluteUnbounded

  typedef JointRevoluteUnbounded<0> JointRUBX;
  typedef JointDataRevoluteUnbounded<0> JointDataRUBX;
  typedef JointModelRevoluteUnbounded<0> JointModelRUBX;

  template<> inline
  const std::string JointModelRevoluteUnbounded<0>::shortname()
  {
    return std::string("JointModelRUBX") ;
  }

  typedef JointRevoluteUnbounded<1> JointRUBY;
  typedef JointDataRevoluteUnbounded<1> JointDataRUBY;
  typedef JointModelRevoluteUnbounded<1> JointModelRUBY;

  template<> inline
  const std::string JointModelRevoluteUnbounded<1>::shortname()
  {
    return std::string("JointModelRUBY") ;
  }

  typedef JointRevoluteUnbounded<2> JointRUBZ;
  typedef JointDataRevoluteUnbounded<2> JointDataRUBZ;
  typedef JointModelRevoluteUnbounded<2> JointModelRUBZ;

  template<> inline
  const std::string JointModelRevoluteUnbounded<2>::shortname()
  {
    return std::string("JointModelRUBZ") ;
  }

} //namespace se3

#endif // ifndef __se3_joint_revolute_unbounded_hpp__
