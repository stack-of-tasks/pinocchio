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

#ifndef __se3_joint_revolute_hpp__
#define __se3_joint_revolute_hpp__

#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/constraint.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/spatial/spatial-axis.hpp"
#include "pinocchio/utils/axis-label.hpp"

namespace se3
{

  template<typename Scalar, int Options, int axis> struct MotionRevoluteTpl;

  namespace revolute
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
  } // namespace revolute


  template<typename _Scalar, int _Options, int axis>
  struct traits< MotionRevoluteTpl<_Scalar,_Options,axis> >
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
  }; // traits MotionRevoluteTpl

  template<typename _Scalar, int _Options, int axis>
  struct MotionRevoluteTpl : MotionBase< MotionRevoluteTpl<_Scalar,_Options,axis> >
  {
    MOTION_TYPEDEF_TPL(MotionRevoluteTpl);
    typedef SpatialAxis<axis+ANGULAR> Axis;

    MotionRevoluteTpl()                   : w(NAN) {}
    
    template<typename OtherScalar>
    MotionRevoluteTpl(const OtherScalar & w) : w(w)  {}
    
    operator MotionPlain() const { return Axis() * w; }
    
    template<typename MotionDerived>
    void addTo(MotionDense<MotionDerived> & v) const
    {
      typedef typename MotionDense<MotionDerived>::Scalar OtherScalar;
      v.angular()[axis] += (OtherScalar)w;
    }
    
    // data
    Scalar w;
  }; // struct MotionRevoluteTpl

  template<typename S1, int O1, int axis, typename MotionDerived>
  typename MotionDerived::MotionPlain
  operator+(const MotionRevoluteTpl<S1,O1,axis> & m1,
            const MotionDense<MotionDerived> & m2)
  {
    typename MotionDerived::MotionPlain res(m2);
    res += m1;
    return res;
  }

  template<typename Scalar, int Options, int axis> struct ConstraintRevoluteTpl;
  
  namespace internal
  {
    template<typename Scalar, int Options, int axis>
    struct SE3GroupAction< ConstraintRevoluteTpl<Scalar,Options,axis> >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
    
    template<typename Scalar, int Options, int axis, typename MotionDerived>
    struct MotionAlgebraAction< ConstraintRevoluteTpl<Scalar,Options,axis>, MotionDerived >
    { typedef Eigen::Matrix<Scalar,6,1,Options> ReturnType; };
  }

  template<typename _Scalar, int _Options, int axis>
  struct traits< ConstraintRevoluteTpl<_Scalar,_Options,axis> >
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
    typedef Eigen::Quaternion<Scalar,0> Quaternion_t;
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
  }; // traits ConstraintRevoluteTpl

  template<typename _Scalar, int _Options, int axis>
  struct ConstraintRevoluteTpl : ConstraintBase< ConstraintRevoluteTpl<_Scalar,_Options,axis> >
  { 
    SPATIAL_TYPEDEF_TEMPLATE(ConstraintRevoluteTpl);
    enum { NV = 1, Options = 0 };
    typedef typename traits<ConstraintRevoluteTpl>::JointMotion JointMotion;
    typedef typename traits<ConstraintRevoluteTpl>::JointForce JointForce;
    typedef typename traits<ConstraintRevoluteTpl>::DenseBase DenseBase;
    typedef SpatialAxis<ANGULAR+axis> Axis;

    template<typename Vector1Like>
    MotionRevoluteTpl<Scalar,Options,axis>
    operator*(const Eigen::MatrixBase<Vector1Like> & v) const
    { return MotionRevoluteTpl<Scalar,Options,axis>(v[0]); }

    template<typename S1, int O1>
    Eigen::Matrix<Scalar,6,1,Options>
    se3Action(const SE3Tpl<S1,O1> & m) const
    { 
      Eigen::Matrix<Scalar,6,1,Options> res;
      res.template head<3>() = m.translation().cross(m.rotation().col(axis));
      res.template tail<3>() = m.rotation().col(axis);
      return res;
    }

    int nv_impl() const { return NV; }
    
    struct TransposeConst
    {
      const ConstraintRevoluteTpl & ref;
      TransposeConst(const ConstraintRevoluteTpl & ref) : ref(ref) {}

      template<typename Derived>
      typename ForceDense<Derived>::ConstAngularType::template ConstFixedSegmentReturnType<1>::Type
      operator* (const ForceDense<Derived> & f) const
      { return f.angular().template segment<1>(axis); }

        /// [CRBA]  MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
      template<typename D>
      typename Eigen::MatrixBase<D>::ConstRowXpr
      operator*(const Eigen::MatrixBase<D> & F) const
      {
        assert(F.rows()==6);
        return F.row(ANGULAR + axis);
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
  }; // struct ConstraintRevoluteTpl

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteTpl<S2,O2,0>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(w,0,0) = ( 0,zw,-yw )
     * nu1^(0,wx) = ( 0,vz1 wx,-vy1 wx,    0,wz1 wx,-wy1 wx)
     */
    typedef typename MotionDerived::MotionPlain ReturnType;
    const typename MotionDerived::ConstLinearType & v = m1.linear();
    const typename MotionDerived::ConstAngularType & w = m1.angular();
    const S2 & wx = m2.w;
    return ReturnType(typename ReturnType::Vector3(0,v[2]*wx,-v[1]*wx),
                      typename ReturnType::Vector3(0,w[2]*wx,-w[1]*wx)
                      );
  }

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteTpl<S2,O2,1>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,w,0) = ( -z,0,x )
     * nu1^(0,wx) = ( -vz1 wx,0,vx1 wx,    -wz1 wx,0,wx1 wx)
     */
    typedef typename MotionDerived::MotionPlain ReturnType;
    const typename MotionDerived::ConstLinearType & v = m1.linear();
    const typename MotionDerived::ConstAngularType & w = m1.angular();
    const S2 & wx = m2.w;
    return ReturnType(typename ReturnType::Vector3(-v[2]*wx,0, v[0]*wx),
                      typename ReturnType::Vector3(-w[2]*wx,0, w[0]*wx)
                      );
  }

  template<typename MotionDerived, typename S2, int O2>
  inline typename MotionDerived::MotionPlain
  operator^(const MotionDense<MotionDerived> & m1, const MotionRevoluteTpl<S2,O2,2>& m2)
  {
    /* nu1^nu2    = ( v1^w2+w1^v2, w1^w2 )
     * nu1^(0,w2) = ( v1^w2      , w1^w2 )
     * (x,y,z)^(0,0,w) = ( y,-x,0 )
     * nu1^(0,wx) = ( vy1 wx,-vx1 wx,0,    wy1 wx,-wx1 wx,0 )
     */
    typedef typename MotionDerived::MotionPlain ReturnType;
    const typename MotionDerived::ConstLinearType & v = m1.linear();
    const typename MotionDerived::ConstAngularType & w = m1.angular();
    const S2 & wx = m2.w;
    return ReturnType(typename ReturnType::Vector3(v[1]*wx,-v[0]*wx,0),
                      typename ReturnType::Vector3(w[1]*wx,-w[0]*wx,0)
                      );
  }

  template<typename Scalar, int Options, int axis>
  struct JointRevoluteTpl
  {
    template<typename S1, typename S2, typename Matrix3Like>
    static void cartesianRotation(const S1 & ca, const S2 & sa,
                                  const Eigen::MatrixBase<Matrix3Like> & res);
  };
  
  template<typename Scalar, int Options>
  struct JointRevoluteTpl<Scalar,Options,0>
  {
    template<typename S1, typename S2, typename Matrix3Like>
    static void cartesianRotation(const S1 & ca, const S2 & sa,
                                  const Eigen::MatrixBase<Matrix3Like> & res)
    {
      Matrix3Like & res_ = const_cast<Matrix3Like &>(res.derived());
      typedef typename Matrix3Like::Scalar OtherScalar;
      res_ <<
      OtherScalar(1), OtherScalar(0), OtherScalar(0),
      OtherScalar(0),             ca,            -sa,
      OtherScalar(0),             sa,             ca;
    }
  };
  
  template<typename Scalar, int Options>
  struct JointRevoluteTpl<Scalar,Options,1>
  {
    template<typename S1, typename S2, typename Matrix3Like>
    static void cartesianRotation(const S1 & ca, const S2 & sa,
                                  const Eigen::MatrixBase<Matrix3Like> & res)
    {
      Matrix3Like & res_ = const_cast<Matrix3Like &>(res.derived());
      typedef typename Matrix3Like::Scalar OtherScalar;
      res_ <<
                  ca, OtherScalar(0),               sa,
      OtherScalar(0), OtherScalar(1),   OtherScalar(0),
                 -sa, OtherScalar(0),               ca;
    }
  };
  
  template<typename Scalar, int Options>
  struct JointRevoluteTpl<Scalar,Options,2>
  {
    template<typename S1, typename S2, typename Matrix3Like>
    static void cartesianRotation(const S1 & ca, const S2 & sa,
                                  const Eigen::MatrixBase<Matrix3Like> & res)
    {
      Matrix3Like & res_ = const_cast<Matrix3Like &>(res.derived());
      typedef typename Matrix3Like::Scalar OtherScalar;
      res_ <<
                  ca,            -sa, OtherScalar(0),
                  sa,             ca, OtherScalar(0),
      OtherScalar(0), OtherScalar(0), OtherScalar(1);
    }
  };

  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,1,O2>
  operator*(const InertiaTpl<S1,O1> & Y,const ConstraintRevoluteTpl<S2,O2,0> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    /* Y(:,3) = ( 0,-z, y,  I00+yy+zz,  I01-xy   ,  I02-xz   ) */
    const S1
    &m = Y.mass(),
    &x = Y.lever()[0],
    &y = Y.lever()[1],
    &z = Y.lever()[2];
    const typename Inertia::Symmetric3 & I = Y.inertia();
    
    Eigen::Matrix<S2,6,1,O2> res;
    res << 0.0,-m*z,m*y,
    I(0,0)+m*(y*y+z*z),
    I(0,1)-m*x*y,
    I(0,2)-m*x*z ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,1,O2>
  operator*(const InertiaTpl<S1,O1> & Y,const ConstraintRevoluteTpl<S2,O2,1> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    /* Y(:,4) = ( z, 0,-x,  I10-xy   ,  I11+xx+zz,  I12-yz   ) */
    const S1
    &m = Y.mass(),
    &x = Y.lever()[0],
    &y = Y.lever()[1],
    &z = Y.lever()[2];
    const typename Inertia::Symmetric3 & I = Y.inertia();
    Eigen::Matrix<S2,6,1,O2> res;
    res << m*z,0,-m*x,
    I(1,0)-m*x*y,
    I(1,1)+m*(x*x+z*z),
    I(1,2)-m*y*z ;
    return res;
  }
  /* [CRBA] ForceSet operator* (Inertia Y,Constraint S) */
  template<typename S1, int O1, typename S2, int O2>
  inline Eigen::Matrix<S2,6,1,O2>
  operator*(const InertiaTpl<S1,O1> & Y,const ConstraintRevoluteTpl<S2,O2,2> &)
  {
    typedef InertiaTpl<S1,O1> Inertia;
    /* Y(:,5) = (-y, x, 0,  I20-xz   ,  I21-yz   ,  I22+xx+yy) */
    const S1
    &m = Y.mass(),
    &x = Y.lever()[0],
    &y = Y.lever()[1],
    &z = Y.lever()[2];
    const typename Inertia::Symmetric3 & I = Y.inertia();
    Eigen::Matrix<S2,6,1,O2> res; res << -m*y,m*x,0,
    I(2,0)-m*x*z,
    I(2,1)-m*y*z,
    I(2,2)+m*(x*x+y*y) ;
    return res;
  }
  
  /* [ABA] I*S operator (Inertia Y,Constraint S) */
  template<typename M6Like, typename S2, int O2,int axis>
  inline const typename M6Like::ConstColXpr
  operator*(const Eigen::MatrixBase<M6Like> & Y, const ConstraintRevoluteTpl<S2,O2,axis> &)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M6Like,6,6);
    return Y.col(Inertia::ANGULAR + axis);
  }

  template<typename _Scalar, int _Options, int axis>
  struct traits< JointRevoluteTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 1,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataRevoluteTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelRevoluteTpl<Scalar,Options,axis> JointModelDerived;
    typedef ConstraintRevoluteTpl<Scalar,Options,axis> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionRevoluteTpl<Scalar,Options,axis> Motion_t;
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
  struct traits< JointDataRevoluteTpl<Scalar,Options,axis> >
  { typedef JointRevoluteTpl<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelRevoluteTpl<Scalar,Options,axis> >
  { typedef JointRevoluteTpl<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataRevoluteTpl : public JointDataBase< JointDataRevoluteTpl<_Scalar,_Options,axis> >
  {
    typedef JointRevoluteTpl<_Scalar,_Options,axis> JointDerived;
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

    JointDataRevoluteTpl() : M(1), U(), Dinv(), UDinv()
    {}

  }; // struct JointDataRevoluteTpl

  template<typename _Scalar, int _Options, int axis>
  struct JointModelRevoluteTpl : public JointModelBase< JointModelRevoluteTpl<_Scalar,_Options,axis> >
  {
    typedef JointRevoluteTpl<_Scalar,_Options,axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;

    using JointModelBase<JointModelRevoluteTpl>::id;
    using JointModelBase<JointModelRevoluteTpl>::idx_q;
    using JointModelBase<JointModelRevoluteTpl>::idx_v;
    using JointModelBase<JointModelRevoluteTpl>::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      typedef typename ConfigVector::Scalar OtherScalar;
      
      const OtherScalar & q = qs[idx_q()];
      OtherScalar ca,sa; SINCOS(q,&sa,&ca);
      JointDerived::cartesianRotation(ca,sa,data.M.rotation());
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(TangentVector_t,TangentVector);
      calc(data,qs.derived());

      data.v.w = (Scalar)vs[idx_v()];;
    }
    
    template<typename S2, int O2>
    void calc_aba(JointDataDerived & data, Eigen::Matrix<S2,6,6,O2> & I, const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = Scalar(1)/I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using std::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname()
    {
      return std::string("JointModelR") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }

  }; // struct JointModelRevoluteTpl

  typedef JointRevoluteTpl<double,0,0> JointRX;
  typedef JointDataRevoluteTpl<double,0,0> JointDataRX;
  typedef JointModelRevoluteTpl<double,0,0> JointModelRX;

  typedef JointRevoluteTpl<double,0,1> JointRY;
  typedef JointDataRevoluteTpl<double,0,1> JointDataRY;
  typedef JointModelRevoluteTpl<double,0,1> JointModelRY;

  typedef JointRevoluteTpl<double,0,2> JointRZ;
  typedef JointDataRevoluteTpl<double,0,2> JointDataRZ;
  typedef JointModelRevoluteTpl<double,0,2> JointModelRZ;

} //namespace se3

#endif // ifndef __se3_joint_revolute_hpp__
