//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015-2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"

#include <stdexcept>

namespace se3
{
  
  namespace prismatic
  {
    template<int axis>
    struct CartesianVector3
    {
      double v; 
      CartesianVector3(const double & v) : v(v) {}
      CartesianVector3() : v(NAN) {}
      
      Eigen::Vector3d vector() const;
      operator Eigen::Vector3d () const { return vector(); }
    }; // struct CartesianVector3
    template<> inline Eigen::Vector3d CartesianVector3<0>::vector() const { return Eigen::Vector3d(v,0,0); }
    template<> inline Eigen::Vector3d CartesianVector3<1>::vector() const { return Eigen::Vector3d(0,v,0); }
    template<> inline Eigen::Vector3d CartesianVector3<2>::vector() const { return Eigen::Vector3d(0,0,v); }
    
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<0> & vx)
    { return Eigen::Vector3d(v1[0]+vx.v,v1[1],v1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<1> & vy)
    { return Eigen::Vector3d(v1[0],v1[1]+vy.v,v1[2]); }
    inline Eigen::Vector3d operator+ (const Eigen::Vector3d & v1,const CartesianVector3<2> & vz)
    { return Eigen::Vector3d(v1[0],v1[1],v1[2]+vz.v); }
  } // namespace prismatic

  template<typename _Scalar, int _axis> struct MotionPrismatic;
  template<typename _Scalar, int _axis>
  struct traits < MotionPrismatic<_Scalar,_axis> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,3,1,0> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,0> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,0> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef MotionTpl<Scalar,0> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // struct traits MotionPrismatic

  template<typename _Scalar, int _axis>
  struct MotionPrismatic : MotionBase < MotionPrismatic<_Scalar,_axis> >
  {
    MOTION_TYPEDEF_TPL(MotionPrismatic);
    typedef SpatialAxis<_axis+LINEAR> Axis;

    MotionPrismatic()                   : v(NAN) {}
    MotionPrismatic( const Scalar & v ) : v(v)  {}
    Scalar v;

    inline operator MotionPlain() const { return Axis() * v; }
    
    template<typename Derived>
    void addTo(MotionDense<Derived> & v_) const
    {
      typedef typename MotionDense<Derived>::Scalar OtherScalar;
      v_.linear()[_axis] += (OtherScalar) v;
    }
  }; // struct MotionPrismatic

  template<typename Scalar, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionPrismatic<Scalar,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<int axis> struct ConstraintPrismatic;
  template<int axis>
  struct traits< ConstraintPrismatic<axis> >
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
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
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
    MotionPrismatic<Scalar,axis> operator*( const Eigen::MatrixBase<D> & v ) const
    {
//        EIGEN_STATIC_ASSERT_SIZE_1x1(D); // There is actually a bug in Eigen with such a macro
      assert(v.cols() == 1 && v.rows() == 1);
      return MotionPrismatic<Scalar,axis>(v[0]);
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

      template<typename Derived>
      typename ForceDense<Derived>::ConstLinearType::template ConstFixedSegmentReturnType<1>::Type
      operator* (const ForceDense<Derived> & f) const
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
    DenseBase matrix_impl() const
    {
      DenseBase S;
      S << prismatic::CartesianVector3<axis>(1).vector(), Eigen::Vector3d::Zero();
      return S;
    }
    
    DenseBase motionAction(const Motion & m) const
    {
      DenseBase res;
      res << m.angular().cross(prismatic::CartesianVector3<axis>(1).vector()), Vector3::Zero();
      
      return res;
    }

  }; // struct ConstraintPrismatic





  template<int axis> 
  struct JointPrismatic
  {
    static Eigen::Vector3d cartesianTranslation(const double & shift); 
  };

  template<typename MotionDerived, typename Scalar>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<Scalar,0>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(v,0,0) = ( 0,zv,-yv )
     * nu1^(0,vx) = ( 0,wz1 vx,-wy1 vx,    0, 0, 0)
     */
    typedef typename MotionDerived::MotionPlain MotionPlain;
    const typename MotionDerived::ConstAngularType & w = m1.angular();
    const Scalar & vx = m2.v;
    return MotionPlain(MotionPlain::Vector3(0,w[2]*vx,-w[1]*vx),
                       MotionPlain::Vector3::Zero());
   }

  template<typename MotionDerived, typename Scalar>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<Scalar,1>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,v,0) = ( -zv,0,xv )
     * nu1^(0,vx) = ( -vz1 vx,0,vx1 vx,    0, 0, 0)
     */
     typedef typename MotionDerived::MotionPlain MotionPlain;
     const typename MotionDerived::ConstAngularType & w = m1.angular();
     const Scalar & vx = m2.v;
     return MotionPlain(MotionPlain::Vector3(-w[2]*vx,0,w[0]*vx),
                        MotionPlain::Vector3::Zero());
   }

  template<typename MotionDerived, typename Scalar>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<Scalar,2>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,0,v) = ( yv,-xv,0 )
     * nu1^(0,vx) = ( vy1 vx,-vx1 vx, 0,    0, 0, 0 )
     */
     typedef typename MotionDerived::MotionPlain MotionPlain;
     const typename MotionDerived::ConstAngularType & w = m1.angular();
     const Scalar & vx = m2.v;
     return MotionPlain(Motion::Vector3(w[1]*vx,-w[0]*vx,0),
                        MotionPlain::Vector3::Zero());
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
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  template<int axis>
  inline const Inertia::Matrix6::ConstColXpr
  operator*(const Inertia::Matrix6 & Y, const ConstraintPrismatic<axis> & )
  {
    return Y.col(Inertia::LINEAR + axis);
  }

  namespace internal 
  {
    template<int axis>
    struct SE3GroupAction< ConstraintPrismatic<axis> >
    { typedef Eigen::Matrix<double,6,1> ReturnType; };
    
    template<int axis>
    struct MotionAlgebraAction< ConstraintPrismatic<axis> >
    { typedef Eigen::Matrix<double,6,1> ReturnType; };
  }



  template<int axis>
  struct traits< JointPrismatic<axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef double Scalar;
    typedef JointDataPrismatic<axis> JointDataDerived;
    typedef JointModelPrismatic<axis> JointModelDerived;
    typedef ConstraintPrismatic<axis> Constraint_t;
    typedef SE3 Transformation_t;
    typedef MotionPrismatic<Scalar,axis> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<double,6,NV> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<double,6,NV> U_t;
    typedef Eigen::Matrix<double,NV,NV> D_t;
    typedef Eigen::Matrix<double,6,NV> UD_t;

    typedef Eigen::Matrix<double,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<double,NV,1> TangentVector_t;
  };

  template<int axis> struct traits< JointDataPrismatic<axis> > { typedef JointPrismatic<axis> JointDerived; };
  template<int axis> struct traits< JointModelPrismatic<axis> > { typedef JointPrismatic<axis> JointDerived; };

  template<int axis>
  struct JointDataPrismatic : public JointDataBase< JointDataPrismatic<axis> >
  {
    typedef JointPrismatic<axis> JointDerived;
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

    JointDataPrismatic() : M(1), U(), Dinv(), UDinv()
    {}

  }; // struct JointDataPrismatic

  template<int axis>
  struct JointModelPrismatic : public JointModelBase< JointModelPrismatic<axis> >
  {
    typedef JointPrismatic<axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelPrismatic>::id;
    using JointModelBase<JointModelPrismatic>::idx_q;
    using JointModelBase<JointModelPrismatic>::idx_v;
    using JointModelBase<JointModelPrismatic>::setIndexes;
    typedef Motion::Vector3 Vector3;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    void calc( JointDataDerived& data, 
      const Eigen::VectorXd & qs ) const
    {
      const double & q = qs[idx_q()];
      data.M.translation(JointPrismatic<axis>::cartesianTranslation(q));
    }

    void calc( JointDataDerived& data, 
      const Eigen::VectorXd & qs, 
      const Eigen::VectorXd & vs ) const
    {
      const double & q = qs[idx_q()];
      const double & v = vs[idx_v()];

      data.M.translation(JointPrismatic<axis>::cartesianTranslation(q));
      data.v.v = v;
    }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U = I.col(Inertia::LINEAR + axis);
      data.Dinv[0] = 1./I(Inertia::LINEAR + axis, Inertia::LINEAR + axis);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return sqrt(Eigen::NumTraits<Scalar>::epsilon());
    }

    static std::string classname();
    std::string shortname() const { return classname(); }

  }; // struct JointModelPrismatic

  typedef JointPrismatic<0> JointPX;
  typedef JointDataPrismatic<0> JointDataPX;
  typedef JointModelPrismatic<0> JointModelPX;

  template<> inline
  std::string JointModelPrismatic<0>::classname()
  {
    return std::string("JointModelPX");
  }

  typedef JointPrismatic<1> JointPY;
  typedef JointDataPrismatic<1> JointDataPY;
  typedef JointModelPrismatic<1> JointModelPY;

  template<> inline
  std::string JointModelPrismatic<1>::classname()
  {
    return std::string("JointModelPY");
  }

  typedef JointPrismatic<2> JointPZ;
  typedef JointDataPrismatic<2> JointDataPZ;
  typedef JointModelPrismatic<2> JointModelPZ;

  template<> inline
  std::string JointModelPrismatic<2>::classname()
  {
    return std::string("JointModelPZ");
  }

} //namespace se3

#endif // ifndef __se3_joint_prismatic_hpp__
