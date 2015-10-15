//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_joint_prismatic_hpp__
#define __se3_joint_prismatic_hpp__

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-dense.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace se3
{

  template<int axis> struct JointDataPrismatic;
  template<int axis> struct JointModelPrismatic;
  
  namespace prismatic
  {
    template<int axis>
    struct CartesianVector3
    {
      double v; 
      CartesianVector3(const double & v) : v(v) {}
      CartesianVector3() : v(NAN) {}
      operator Eigen::Vector3d () const; 
    }; // struct CartesianVector3
    template<> inline CartesianVector3<0>::operator Eigen::Vector3d () const { return Eigen::Vector3d(v,0,0); }
    template<> inline CartesianVector3<1>::operator Eigen::Vector3d () const { return Eigen::Vector3d(0,v,0); }
    template<> inline CartesianVector3<2>::operator Eigen::Vector3d () const { return Eigen::Vector3d(0,0,v); }
    
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<0> & vx)
    { return Eigen::Vector3d(v1[0]+vx.v,v1[1],v1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<1> & vy)
    { return Eigen::Vector3d(v1[0],v1[1]+vy.v,v1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<2> & vz)
    { return Eigen::Vector3d(v1[0],v1[1],v1[2]+vz.v); }
  } // namespace prismatic

  template <int axis> struct MotionPrismatic;
  template<int axis>
  struct traits <MotionPrismatic < axis > >
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
  }; // struct traits MotionPrismatic

  template<int axis>
  struct MotionPrismatic : MotionBase < MotionPrismatic < axis > >
  {
    SPATIAL_TYPEDEF_TEMPLATE(MotionPrismatic);

    MotionPrismatic()                   : v(NAN) {}
    MotionPrismatic( const double & v ) : v(v)  {}
    double v;

    operator Motion() const
    { 
      return Motion((Vector3)typename prismatic::CartesianVector3<axis>(v),
                      Motion::Vector3::Zero()
                    );
    }
  }; // struct MotionPrismatic

  template <int axis>
  const MotionPrismatic<axis>& operator+ (const MotionPrismatic<axis>& m, const BiasZero&)
  { return m; }

  template <int axis>
  Motion operator+( const MotionPrismatic<axis>& m1, const Motion& m2)
  {
    return Motion( m2.linear()+typename prismatic::CartesianVector3<axis>(m1.v),m2.angular()); 
  }

  template<int axis> struct ConstraintPrismatic;
  template<int axis>
  struct traits< ConstraintPrismatic<axis> >
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

  template <int axis>
  struct ConstraintPrismatic : ConstraintBase < ConstraintPrismatic <axis > >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintPrismatic);
    enum { NV = 1, Options = 0 };
    typedef typename traits<ConstraintPrismatic>::JointMotion JointMotion;
    typedef typename traits<ConstraintPrismatic>::JointForce JointForce;
    typedef typename traits<ConstraintPrismatic>::DenseBase DenseBase;

    template<typename D>
    MotionPrismatic<axis> operator*( const Eigen::MatrixBase<D> & v ) const
    {
//        EIGEN_STATIC_ASSERT_SIZE_1x1(D); // There is actually a bug in Eigen with such a macro
      return MotionPrismatic<axis>(v[0]);
    }

    Eigen::Matrix<double,6,1> se3Action(const SE3 & m) const
    { 
     Eigen::Matrix<double,6,1> res;
     res.head<3>() = m.rotation().col(axis);
     res.tail<3>() = Motion::Vector3::Zero();
     return res;
    }

    int nv_impl() const { return NV; }

    struct TransposeConst
    {
      const ConstraintPrismatic & ref; 
      TransposeConst(const ConstraintPrismatic<axis> & ref) : ref(ref) {} 

      typename Force::Vector3::template ConstFixedSegmentReturnType<1>::Type
      operator*( const Force& f ) const
      { return f.linear().template segment<1>(axis); }

      /* [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block) */
      template<typename D>
      friend typename Eigen::MatrixBase<D>::ConstRowXpr
      operator*( const TransposeConst &, const Eigen::MatrixBase<D> & F )
      {
        assert(F.rows()==6);
        return F.row(axis);
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
      S << (Eigen::Vector3d)prismatic::CartesianVector3<axis>(), Eigen::Vector3d::Zero() ;
      return ConstraintXd(S);
    }

  }; // struct ConstraintPrismatic





  template<int axis> 
  struct JointPrismatic
  {
    static Eigen::Vector3d cartesianTranslation(const double & shift); 
  };

  inline Motion operator^( const Motion& m1, const MotionPrismatic<0>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(v,0,0) = ( 0,zv,-yv )
     * nu1^(0,vx) = ( 0,wz1 vx,-wy1 vx,    0, 0, 0)
     */
     const Motion::Vector3& w = m1.angular();
     const double & vx = m2.v;
     return Motion( Motion::Vector3(0,w[2]*vx,-w[1]*vx),
       Motion::Vector3::Zero());
   }

   inline Motion operator^( const Motion& m1, const MotionPrismatic<1>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,v,0) = ( -zv,0,xv )
     * nu1^(0,vx) = ( -vz1 vx,0,vx1 vx,    0, 0, 0)
     */
     const Motion::Vector3& w = m1.angular();
     const double & vx = m2.v;
     return Motion( Motion::Vector3(-w[2]*vx,0,w[0]*vx),
       Motion::Vector3::Zero());
   }

   inline Motion operator^( const Motion& m1, const MotionPrismatic<2>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,0,v) = ( yv,-xv,0 )
     * nu1^(0,vx) = ( vy1 vx,-vx1 vx, 0,    0, 0, 0 )
     */
     const Motion::Vector3& w = m1.angular();
     const double & vx = m2.v;
     return Motion( Motion::Vector3(w[1]*vx,-w[0]*vx,0),
       Motion::Vector3::Zero());
   }

  template<> inline
  Eigen::Vector3d JointPrismatic<0>::cartesianTranslation(const double & shift) 
  {
    return Motion::Vector3(shift,0,0);
  }
  template<> inline
  Eigen::Vector3d JointPrismatic<1>::cartesianTranslation(const double & shift) 
  {
    return Motion::Vector3(0,shift,0);
  }
  template<> inline
  Eigen::Vector3d JointPrismatic<2>::cartesianTranslation(const double & shift) 
  {
    return Motion::Vector3(0,0,shift);
  }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const ConstraintPrismatic<0> & )
  { 
    /* Y(:,0) = ( 1,0, 0, 0 , z , -y ) */
    const double 
    &m = Y.mass(),
    &y = Y.lever()[1],
    &z = Y.lever()[2];
    Eigen::Matrix<double,6,1> res; res << m,0.0,0.0,
    0.0,
    m*z,
    -m*y ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const ConstraintPrismatic<1> & )
  { 
    /* Y(:,1) = ( 0,1, 0, -z , 0 , x) */
    const double 
    &m = Y.mass(),
    &x = Y.lever()[0],
    &z = Y.lever()[2];
    Eigen::Matrix<double,6,1> res; res << 0.0,m,0.0,
    -m*z,
    0.0,
    m*x ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  inline Eigen::Matrix<double,6,1>
  operator*( const Inertia& Y,const ConstraintPrismatic<2> & )
  { 
    /* Y(:,2) = ( 0,0, 1, y , -x , 0) */
    const double 
    &m = Y.mass(),
    &x = Y.lever()[0],
    &y = Y.lever()[1];
    Eigen::Matrix<double,6,1> res; res << 0.0,0.0,m,
    m*y,
    -m*x,
    0.0;
    return res;
  }

  namespace internal 
  {
    template<int axis>
    struct ActionReturn<ConstraintPrismatic<axis> >
    { typedef Eigen::Matrix<double,6,1> Type; };
  }



  template<int axis>
  struct traits< JointPrismatic<axis> >
  {
    typedef JointDataPrismatic<axis> JointData;
    typedef JointModelPrismatic<axis> JointModel;
    typedef ConstraintPrismatic<axis> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionPrismatic<axis> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,1> F_t;
    enum {
      NQ = 1,
      NV = 1
    };
  };

  template<int axis> struct traits< JointDataPrismatic<axis> > { typedef JointPrismatic<axis> Joint; };
  template<int axis> struct traits< JointModelPrismatic<axis> > { typedef JointPrismatic<axis> Joint; };

  template<int axis>
  struct JointDataPrismatic : public JointDataBase< JointDataPrismatic<axis> >
  {
    typedef JointPrismatic<axis> Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    F_t F;

    JointDataPrismatic() : M(1) // Etat initial de la liaison ?
    {
      M.rotation(SE3::Matrix3::Identity());
    }

    JointDataDense<NQ, NV> toDense_impl() const
    {
      return JointDataDense<NQ, NV>(S, M, v, c, F);
    }

  }; // struct JointDataPrismatic

  template<int axis>
  struct JointModelPrismatic : public JointModelBase< JointModelPrismatic<axis> >
  {
    typedef JointPrismatic<axis> Joint;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelPrismatic>::id;
    using JointModelBase<JointModelPrismatic>::idx_q;
    using JointModelBase<JointModelPrismatic>::idx_v;
    using JointModelBase<JointModelPrismatic>::lowerPosLimit;
    using JointModelBase<JointModelPrismatic>::upperPosLimit;
    using JointModelBase<JointModelPrismatic>::maxEffortLimit;
    using JointModelBase<JointModelPrismatic>::maxVelocityLimit;
    using JointModelBase<JointModelPrismatic>::setIndexes;
    
    JointData createData() const { return JointData(); }
    void calc( JointData& data, 
      const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      data.M.translation(JointPrismatic<axis>::cartesianTranslation(q));
    }

    void calc( JointData& data, 
      const Eigen::VectorXd & qs, 
      const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.translation(JointPrismatic<axis>::cartesianTranslation(q));
      data.v.v = v;
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

    static const std::string shortname();

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelPrismatic> & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v()
              && jmodel.lowerPosLimit() == lowerPosLimit()
              && jmodel.upperPosLimit() == upperPosLimit()
              && jmodel.maxEffortLimit() == maxEffortLimit()
              && jmodel.maxVelocityLimit() == maxVelocityLimit();
    }
  }; // struct JointModelPrismatic

  typedef JointPrismatic<0> JointPX;
  typedef JointDataPrismatic<0> JointDataPX;
  typedef JointModelPrismatic<0> JointModelPX;

  template<> inline
  const std::string JointModelPrismatic<0>::shortname()
  {
    return std::string("JointModelPX");
  }

  typedef JointPrismatic<1> JointPY;
  typedef JointDataPrismatic<1> JointDataPY;
  typedef JointModelPrismatic<1> JointModelPY;

  template<> inline
  const std::string JointModelPrismatic<1>::shortname()
  {
    return std::string("JointModelPY");
  }

  typedef JointPrismatic<2> JointPZ;
  typedef JointDataPrismatic<2> JointDataPZ;
  typedef JointModelPrismatic<2> JointModelPZ;

  template<> inline
  const std::string JointModelPrismatic<2>::shortname()
  {
    return std::string("JointModelPZ");
  }

} //namespace se3

#endif // ifndef __se3_joint_prismatic_hpp__
