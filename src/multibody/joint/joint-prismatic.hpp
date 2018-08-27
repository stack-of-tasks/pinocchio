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
#include "pinocchio/utils/axis-label.hpp"

namespace se3
{
  
  template<typename Scalar, int Options, int _axis> struct MotionPrismatic;
  
  template<typename _Scalar, int _Options, int _axis>
  struct traits < MotionPrismatic<_Scalar,_Options,_axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef typename EIGEN_REF_CONSTTYPE(Vector6) ToVectorConstReturnType;
    typedef typename EIGEN_REF_TYPE(Vector6) ToVectorReturnType;
    typedef Vector3 AngularType;
    typedef Vector3 LinearType;
    typedef const Vector3 ConstAngularType;
    typedef const Vector3 ConstLinearType;
    typedef Matrix6 ActionMatrixType;
    typedef MotionTpl<Scalar,Options> MotionPlain;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // struct traits MotionPrismatic

  template<typename _Scalar, int _Options, int _axis>
  struct MotionPrismatic : MotionBase < MotionPrismatic<_Scalar,_Options,_axis> >
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

  template<typename Scalar, int Options, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionPrismatic<Scalar,Options,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<typename Scalar, int Options, int axis> struct ConstraintPrismatic;
  
  template<typename _Scalar, int _Options, int axis>
  struct traits< ConstraintPrismatic<_Scalar,_Options,axis> >
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,4,1,Options> Vector4;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Matrix3 Angular_t;
    typedef Vector3 Linear_t;
    typedef const Matrix3 ConstAngular_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<Scalar,Options> Quaternion_t;
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef ForceTpl<Scalar,Options> Force;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef Symmetric3Tpl<Scalar,Options> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
    typedef Eigen::Matrix<Scalar,1,1,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,1,1,Options> JointForce;
    typedef Eigen::Matrix<Scalar,6,1,Options> DenseBase;
    typedef DenseBase MatrixReturnType;
    typedef const DenseBase ConstMatrixReturnType;
  }; // traits ConstraintRevolute

  template<typename _Scalar, int _Options, int axis>
  struct ConstraintPrismatic : ConstraintBase < ConstraintPrismatic <_Scalar,_Options,axis> >
  {
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintPrismatic);
    enum { NV = 1, Options = _Options };
    typedef typename traits<ConstraintPrismatic>::JointMotion JointMotion;
    typedef typename traits<ConstraintPrismatic>::JointForce JointForce;
    typedef typename traits<ConstraintPrismatic>::DenseBase DenseBase;
    typedef SpatialAxis<LINEAR+axis> Axis;

    template<typename D>
    MotionPrismatic<Scalar,Options,axis> operator*(const Eigen::MatrixBase<D> & v) const
    {
//        EIGEN_STATIC_ASSERT_SIZE_1x1(D); // There is actually a bug in Eigen with such a macro
      assert(v.cols() == 1 && v.rows() == 1);
      return MotionPrismatic<Scalar,Options,axis>(v[0]);
    }

    template<typename S2, int O2>
    DenseBase se3Action(const SE3Tpl<S2,O2> & m) const
    { 
      DenseBase res;
      MotionRef<DenseBase> v(res);
      v.linear() = m.rotation().col(axis);
      v.angular().setZero();
      return res;
    }

    int nv_impl() const { return NV; }

    struct TransposeConst
    {
      const ConstraintPrismatic & ref; 
      TransposeConst(const ConstraintPrismatic & ref) : ref(ref) {}

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
      MotionRef<DenseBase> v(S);
      v << Axis();
      return S;
    }
    
    template<typename MotionDerived>
    DenseBase motionAction(const MotionDense<MotionDerived> & m) const
    {
      DenseBase res;
      MotionRef<DenseBase> v(res);
      v = m.cross(Axis());
      return res;
    }

  }; // struct ConstraintPrismatic

  template<typename MotionDerived, typename S1, int O1>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<S1,O1,0>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(v,0,0) = ( 0,zv,-yv )
     * nu1^(0,vx) = ( 0,wz1 vx,-wy1 vx,    0, 0, 0)
     */
    typedef typename MotionDerived::MotionPlain MotionPlain;
    const typename MotionDerived::ConstAngularType & w = m1.angular();
    const S1 & vx = m2.v;
    return MotionPlain(typename MotionPlain::Vector3(0,w[2]*vx,-w[1]*vx),
                       MotionPlain::Vector3::Zero());
   }

  template<typename MotionDerived, typename S1, int O1>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<S1,O1,1>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,v,0) = ( -zv,0,xv )
     * nu1^(0,vx) = ( -vz1 vx,0,vx1 vx,    0, 0, 0)
     */
     typedef typename MotionDerived::MotionPlain MotionPlain;
     const typename MotionDerived::ConstAngularType & w = m1.angular();
     const S1 & vy = m2.v;
     return MotionPlain(typename MotionPlain::Vector3(-w[2]*vy,0,w[0]*vy),
                        MotionPlain::Vector3::Zero());
   }

  template<typename MotionDerived, typename S1, int O1>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1,
            const MotionPrismatic<S1,O1,2>& m2)
   {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(v2,0) = ( w1^v2      , 0 )
     * (x,y,z)^(0,0,v) = ( yv,-xv,0 )
     * nu1^(0,vx) = ( vy1 vx,-vx1 vx, 0,    0, 0, 0 )
     */
     typedef typename MotionDerived::MotionPlain MotionPlain;
     const typename MotionDerived::ConstAngularType & w = m1.angular();
     const S1 & vz = m2.v;
     return MotionPlain(typename Motion::Vector3(w[1]*vz,-w[0]*vz,0),
                        MotionPlain::Vector3::Zero());
   }

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S1,6,1,O1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPrismatic<S2,O2,0> &)
  { 
    /* Y(:,0) = ( 1,0, 0, 0 , z , -y ) */
    const S1
    &m = Y.mass(),
    &y = Y.lever()[1],
    &z = Y.lever()[2];
    Eigen::Matrix<S1,6,1,O1> res;
    res << m, S1(0), S1(0), S1(0), m*z, -m*y;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S1,6,1,O1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPrismatic<S2,O2,1> & )
  { 
    /* Y(:,1) = ( 0,1, 0, -z , 0 , x) */
    const S1
    &m = Y.mass(),
    &x = Y.lever()[0],
    &z = Y.lever()[2];
    Eigen::Matrix<S1,6,1,O1> res;
    res << S1(0), m, S1(0), -m*z, S1(0), m*x;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S1,6,1,O1>
  operator*(const InertiaTpl<S1,O1> & Y, const ConstraintPrismatic<S2,O2,2> & )
  { 
    /* Y(:,2) = ( 0,0, 1, y , -x , 0) */
    const S1
    &m = Y.mass(),
    &x = Y.lever()[0],
    &y = Y.lever()[1];
    Eigen::Matrix<S1,6,1,O1> res;
    res << S1(0), S1(0), m, m*y, -m*x, S1(0);
    return res;
  }
  
  /* [ABA] operator* (Inertia Y,Constraint S) */
  template<typename M6Like, typename S2, int O2, int axis>
  inline const typename M6Like::ConstColXpr
  operator*(const Eigen::MatrixBase<M6Like> & Y, const ConstraintPrismatic<S2,O2,axis> &)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
    return Y.derived().col(Inertia::LINEAR + axis);
  }

  namespace internal 
  {
    template<typename Scalar, int Options, int axis>
    struct SE3GroupAction< ConstraintPrismatic<Scalar,Options,axis> >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
    template<typename Scalar, int Options, int axis, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintPrismatic<Scalar,Options,axis>, MotionDerived >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  }
  
  template<typename Scalar, int Options, int axis> struct JointPrismatic {};
 
  
  template<typename Scalar, int Options>
  struct JointPrismatic<Scalar,Options,0>
  {
    template<typename S1, typename S2, int O2>
    static void cartesianTranslation(const S1 & shift, SE3Tpl<S2,O2> & m)
    {
      m.translation() << (S2)(shift), S2(0), S2(0);
    }
  };
  
  template<typename Scalar, int Options>
  struct JointPrismatic<Scalar,Options,1>
  {
    template<typename S1, typename S2, int O2>
    static void cartesianTranslation(const S1 & shift, SE3Tpl<S2,O2> & m)
    {
      m.translation() << S2(0), (S2)(shift), S2(0);
    }
  };
  
  template<typename Scalar, int Options>
  struct JointPrismatic<Scalar,Options,2>
  {
    template<typename S1, typename S2, int O2>
    static void cartesianTranslation(const S1 & shift, SE3Tpl<S2,O2> & m)
    {
      m.translation() << S2(0),  S2(0), (S2)(shift);
    }
  };

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointPrismatic<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataPrismatic<Scalar,Options,axis> JointDataDerived;
    typedef JointModelPrismatic<Scalar,Options,axis> JointModelDerived;
    typedef ConstraintPrismatic<Scalar,Options,axis> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionPrismatic<Scalar,Options,axis> Motion_t;
    typedef BiasZero Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };

  template<typename Scalar, int Options, int axis>
  struct traits< JointDataPrismatic<Scalar,Options,axis> >
  { typedef JointPrismatic<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelPrismatic<Scalar,Options,axis> >
  { typedef JointPrismatic<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataPrismatic : public JointDataBase< JointDataPrismatic<_Scalar,_Options,axis> >
  {
    typedef JointPrismatic<_Scalar,_Options,axis> JointDerived;
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

  template<typename _Scalar, int _Options, int axis>
  struct JointModelPrismatic : public JointModelBase< JointModelPrismatic<_Scalar,_Options,axis> >
  {
    typedef JointPrismatic<_Scalar,_Options,axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelPrismatic>::id;
    using JointModelBase<JointModelPrismatic>::idx_q;
    using JointModelBase<JointModelPrismatic>::idx_v;
    using JointModelBase<JointModelPrismatic>::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(ConfigVector);
      
      typedef typename ConfigVector::Scalar Scalar;
      const Scalar & q = qs[idx_q()];
      JointDerived::cartesianTranslation(q,data.M);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(TangentVector);
      calc(data,qs.derived());
      
      typedef typename TangentVector::Scalar S2;
      const S2 & v = vs[idx_v()];
      data.v.v = v;
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
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

    static std::string classname()
    {
      return std::string("JointModelP") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }

  }; // struct JointModelPrismatic

  typedef JointPrismatic<double,0,0> JointPX;
  typedef JointDataPrismatic<double,0,0> JointDataPX;
  typedef JointModelPrismatic<double,0,0> JointModelPX;

  typedef JointPrismatic<double,0,1> JointPY;
  typedef JointDataPrismatic<double,0,1> JointDataPY;
  typedef JointModelPrismatic<double,0,1> JointModelPY;

  typedef JointPrismatic<double,0,2> JointPZ;
  typedef JointDataPrismatic<double,0,2> JointDataPZ;
  typedef JointModelPrismatic<double,0,2> JointModelPZ;

} //namespace se3

#endif // ifndef __se3_joint_prismatic_hpp__
