//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_joint_revolute_unbounded_hpp__
#define __pinocchio_joint_revolute_unbounded_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, int axis> struct JointRevoluteUnboundedTpl;
  
  template<typename _Scalar, int _Options, int axis>
  struct traits< JointRevoluteUnboundedTpl<_Scalar,_Options,axis> >
  {
    enum {
      NQ = 2,
      NV = 1
    };
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef JointDataRevoluteUnboundedTpl<Scalar,Options,axis> JointDataDerived;
    typedef JointModelRevoluteUnboundedTpl<Scalar,Options,axis> JointModelDerived;
    typedef ConstraintRevoluteTpl<Scalar,Options,axis> Constraint_t;
    typedef TransformRevoluteTpl<Scalar,Options,axis> Transformation_t;
    typedef MotionRevoluteTpl<Scalar,Options,axis> Motion_t;
    typedef BiasZeroTpl<Scalar,Options> Bias_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> F_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,NV,Options> U_t;
    typedef Eigen::Matrix<Scalar,NV,NV,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,NV,Options> UD_t;
    
    PINOCCHIO_JOINT_DATA_BASE_ACCESSOR_DEFAULT_RETURN_TYPE

    typedef Eigen::Matrix<Scalar,NQ,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1,Options> TangentVector_t;
  };

  template<typename Scalar, int Options, int axis>
  struct traits< JointDataRevoluteUnboundedTpl<Scalar,Options,axis> >
  { typedef JointRevoluteUnboundedTpl<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  { typedef JointRevoluteUnboundedTpl<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataRevoluteUnboundedTpl : public JointDataBase< JointDataRevoluteUnboundedTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_DATA_TYPEDEF_TEMPLATE;
    PINOCCHIO_JOINT_DATA_BASE_DEFAULT_ACCESSOR

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;
    F_t F;

    // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

    JointDataRevoluteUnboundedTpl() {}

    static std::string classname() { return std::string("JointDataRevoluteUnbounded"); }
    std::string shortname() const { return classname(); }
    
  }; // struct JointDataRevoluteUnbounded
  
  template<typename NewScalar, typename Scalar, int Options, int axis>
  struct CastType< NewScalar, JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  {
    typedef JointModelRevoluteUnboundedTpl<NewScalar,Options,axis> type;
  };

  template<typename _Scalar, int _Options, int axis>
  struct JointModelRevoluteUnboundedTpl
  : public JointModelBase< JointModelRevoluteUnboundedTpl<_Scalar,_Options,axis> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef JointRevoluteUnboundedTpl<_Scalar,_Options,axis> JointDerived;
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE;
    typedef JointRevoluteTpl<Scalar,_Options,axis> JointDerivedBase;
    
    typedef JointModelBase<JointModelRevoluteUnboundedTpl> Base;
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      typedef typename ConfigVector::Scalar OtherScalar;
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type
      & q = qs.template segment<NQ> (idx_q());

      const OtherScalar & ca = q(0);
      const OtherScalar & sa = q(1);

      data.M.setValues(sa,ca);
    }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs,
              const typename Eigen::MatrixBase<TangentVector> & vs) const
    {
      calc(data,qs.derived());
      
      data.v.w = (Scalar)vs[idx_v()];
    }
    
    template<typename Matrix6Like>
    void calc_aba(JointDataDerived & data, const Eigen::MatrixBase<Matrix6Like> & I, const bool update_I) const
    {
      data.U = I.col(Inertia::ANGULAR + axis);
      data.Dinv[0] = (Scalar)(1)/I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
      data.UDinv.noalias() = data.U * data.Dinv[0];
      
      if (update_I)
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Like,I) -= data.UDinv * data.U.transpose();
    }
    
    Scalar finiteDifferenceIncrement() const
    {
      using math::sqrt;
      return 2.*sqrt(sqrt(Eigen::NumTraits<Scalar>::epsilon()));
    }

    static std::string classname()
    {
      return std::string("JointModelRUB") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }
    
    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    JointModelRevoluteUnboundedTpl<NewScalar,Options,axis> cast() const
    {
      typedef JointModelRevoluteUnboundedTpl<NewScalar,Options,axis> ReturnType;
      ReturnType res;
      res.setIndexes(id(),idx_q(),idx_v());
      return res;
    }

  }; // struct JointModelRevoluteUnboundedTpl
  
  typedef JointRevoluteUnboundedTpl<double,0,0> JointRUBX;
  typedef JointDataRevoluteUnboundedTpl<double,0,0> JointDataRUBX;
  typedef JointModelRevoluteUnboundedTpl<double,0,0> JointModelRUBX;

  typedef JointRevoluteUnboundedTpl<double,0,1> JointRUBY;
  typedef JointDataRevoluteUnboundedTpl<double,0,1> JointDataRUBY;
  typedef JointModelRevoluteUnboundedTpl<double,0,1> JointModelRUBY;

  typedef JointRevoluteUnboundedTpl<double,0,2> JointRUBZ;
  typedef JointDataRevoluteUnboundedTpl<double,0,2> JointDataRUBZ;
  typedef JointModelRevoluteUnboundedTpl<double,0,2> JointModelRUBZ;

} //namespace pinocchio


#include <boost/type_traits.hpp>

namespace boost
{
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_constructor< ::pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
  
  template<typename Scalar, int Options, int axis>
  struct has_nothrow_copy< ::pinocchio::JointDataRevoluteUnboundedTpl<Scalar,Options,axis> >
  : public integral_constant<bool,true> {};
}

#endif // ifndef __pinocchio_joint_revolute_unbounded_hpp__
