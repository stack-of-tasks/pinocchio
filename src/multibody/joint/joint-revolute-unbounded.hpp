//
// Copyright (c) 2016-2018 CNRS
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

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/sincos.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

namespace se3
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
    typedef SE3 Transformation_t;
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
  struct traits< JointDataRevoluteUnboundedTpl<Scalar,Options,axis> >
  { typedef JointRevoluteUnboundedTpl<Scalar,Options,axis> JointDerived; };
  
  template<typename Scalar, int Options, int axis>
  struct traits< JointModelRevoluteUnboundedTpl<Scalar,Options,axis> >
  { typedef JointRevoluteUnboundedTpl<Scalar,Options,axis> JointDerived; };

  template<typename _Scalar, int _Options, int axis>
  struct JointDataRevoluteUnboundedTpl : public JointDataBase< JointDataRevoluteUnboundedTpl<_Scalar,_Options,axis> >
  {
    typedef JointRevoluteUnboundedTpl<_Scalar,_Options,axis> JointDerived;
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

    JointDataRevoluteUnboundedTpl() : M(1), U(), Dinv(), UDinv()
    {}

  }; // struct JointDataRevoluteUnbounded

  template<typename _Scalar, int _Options, int axis>
  struct JointModelRevoluteUnboundedTpl : public JointModelBase< JointModelRevoluteUnboundedTpl<_Scalar,_Options,axis> >
  {
    typedef JointRevoluteUnboundedTpl<_Scalar,_Options,axis> JointDerived;
    SE3_JOINT_TYPEDEF_TEMPLATE;
    typedef JointRevoluteTpl<Scalar,_Options,axis> JointDerivedBase;
    

    using JointModelBase<JointModelRevoluteUnboundedTpl>::id;
    using JointModelBase<JointModelRevoluteUnboundedTpl>::idx_q;
    using JointModelBase<JointModelRevoluteUnboundedTpl>::idx_v;
    using JointModelBase<JointModelRevoluteUnboundedTpl>::setIndexes;
    
    JointDataDerived createData() const { return JointDataDerived(); }
    
    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const typename Eigen::MatrixBase<ConfigVector> & qs) const
    {
      EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(ConfigVector_t,ConfigVector);
      typedef typename ConfigVector::Scalar OtherScalar;
      typename ConfigVector::template ConstFixedSegmentReturnType<NQ>::Type
      & q = qs.template segment<NQ> (idx_q());

      const OtherScalar & ca = q(0);
      const OtherScalar & sa = q(1);

      JointDerivedBase::cartesianRotation(ca,sa,data.M.rotation());
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
      data.Dinv[0] = (Scalar)(1)/I(Inertia::ANGULAR + axis,Inertia::ANGULAR + axis);
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
      return std::string("JointModelRUB") + axisLabel<axis>();
    }
    std::string shortname() const { return classname(); }

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

} //namespace se3

#endif // ifndef __se3_joint_revolute_unbounded_hpp__
