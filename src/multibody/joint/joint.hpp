//
// Copyright (c) 2016,2018 CNRS
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

#ifndef __se3_joint_model_hpp__
#define __se3_joint_model_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hxx"
#include "pinocchio/container/aligned-vector.hpp"

#include <boost/mpl/contains.hpp>

namespace se3
{

  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointTpl;
  typedef JointTpl<double> Joint;

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointTpl<_Scalar,_Options,JointCollectionTpl> >
  {
    enum {
      Options = _Options,
      NQ = Eigen::Dynamic, // Dynamic because unknown at compile time
      NV = Eigen::Dynamic
    };
    
    typedef _Scalar Scalar;
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointDataDerived;
    typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModelDerived;
    
    typedef ConstraintTpl<Eigen::Dynamic,Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTpl<Scalar,Options>  Motion_t;
    typedef MotionTpl<Scalar,Options>  Bias_t;

    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> F_t;
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> U_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> UD_t;

    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> TangentVector_t;
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointDataTpl<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct traits< JointModelTpl<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDataTpl
  : public JointDataBase< JointDataTpl<_Scalar,_Options,JointCollectionTpl> >
  , JointCollectionTpl<_Scalar,_Options>::JointDataVariant
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTpl<_Scalar,_Options,JointCollectionTpl> JointDerived;
    typedef JointDataBase<JointDataTpl> Base;
    
    SE3_JOINT_TYPEDEF_TEMPLATE;

    typedef JointCollectionTpl<_Scalar,_Options> JointCollection;
    typedef typename JointCollection::JointDataVariant JointDataVariant;
    
    JointDataVariant & toVariant() { return *static_cast<JointDataVariant*>(this); }
    const JointDataVariant & toVariant() const { return *static_cast<const JointDataVariant*>(this); }

    const Constraint_t      S() const  { return constraint_xd(*this); }
    const Transformation_t  M() const  { return joint_transform(*this); }
    const Motion_t          v() const  { return motion(*this); }
    const Bias_t            c() const  { return bias(*this); }
    
    // // [ABA CCRBA]
    const U_t               U()     const { return u_inertia(*this); }
    U_t                     U()           { return u_inertia(*this); }
    const D_t               Dinv()  const { return dinv_inertia(*this); }
    const UD_t              UDinv() const { return udinv_inertia(*this); }

    JointDataTpl() : JointDataVariant() {}
    
    JointDataTpl(const JointDataVariant & jdata_variant)
    : JointDataVariant(jdata_variant)
    {}
    
    template<typename JointDataDerived>
    JointDataTpl(const JointDataBase<JointDataDerived> & jdata)
    : JointCollection::JointDataVariant((JointDataVariant)jdata.derived())
    {
      BOOST_MPL_ASSERT((boost::mpl::contains<typename JointDataVariant::types,JointDataDerived>));
    }

  };

  template<typename _Scalar, int _Options, template<typename S, int O> class JointCollectionTpl>
  struct JointModelTpl
  : JointModelBase< JointModelTpl<_Scalar,_Options,JointCollectionTpl> >
  , JointCollectionTpl<_Scalar,_Options>::JointModelVariant
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointTpl<_Scalar,_Options,JointCollectionTpl> JointDerived;

    SE3_JOINT_TYPEDEF_TEMPLATE;
    SE3_JOINT_USE_INDEXES;
    
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef typename JointCollection::JointDataVariant JointDataVariant;
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    
    using Base::id;
    using Base::setIndexes;
    using Base::operator==;

    JointModelTpl() : JointModelVariant() {}
    
    JointModelTpl(const JointModelVariant & jmodel_variant)
    : JointCollection::JointModelVariant(jmodel_variant)
    {}
    
    template<typename JointModelDerived>
    JointModelTpl(const JointModelBase<JointModelDerived> & jmodel)
    : JointModelVariant((JointModelVariant)jmodel.derived())
    {
      BOOST_MPL_ASSERT((boost::mpl::contains<typename JointModelVariant::types,JointModelDerived>));
    }
    
    JointModelVariant & toVariant()
    { return *static_cast<JointModelVariant*>(this); }
    
    const JointModelVariant & toVariant() const
    { return *static_cast<const JointModelVariant*>(this); }

    JointDataDerived createData()
    { return ::se3::createData<JointCollection>(*this); }

    template<typename ConfigVector>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVector> & q) const
    { calc_zero_order(*this,data,q); }

    template<typename ConfigVector, typename TangentVector>
    void calc(JointDataDerived & data,
              const Eigen::MatrixBase<ConfigVector> & q,
              const Eigen::MatrixBase<TangentVector> & v) const
    { calc_first_order(*this,data,q,v); }
    
    void calc_aba(JointDataDerived & data, Inertia::Matrix6 & I, const bool update_I) const
    { ::se3::calc_aba(*this,data,I,update_I); }
    
    std::string shortname() const { return ::se3::shortname(*this); }
    static std::string classname() { return "JointModel"; }

    int     nq_impl() const { return ::se3::nq(*this); }
    int     nv_impl() const { return ::se3::nv(*this); }

    int     idx_q()   const { return ::se3::idx_q(*this); }
    int     idx_v()   const { return ::se3::idx_v(*this); }

    JointIndex     id()      const { return ::se3::id(*this); }

    void setIndexes(JointIndex id, int nq, int nv)
    { ::se3::setIndexes(*this,id, nq, nv); }
  };
  
  typedef container::aligned_vector<JointData> JointDataVector;
  typedef container::aligned_vector<JointModel> JointModelVector;

} // namespace se3

#endif // ifndef __se3_joint_model_hpp__
