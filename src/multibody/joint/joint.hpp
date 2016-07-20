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

#ifndef __se3_joint_model_hpp__
#define __se3_joint_model_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"

namespace se3
{

  struct Joint;
  struct JointModel;
  struct JointData;

  template<>
  struct traits<Joint>
  {

    enum {
      NQ = -1, // Dynamic because unknown at compilation
      NV = -1
    };
    typedef JointData JointDataDerived;
    typedef JointModel JointModelDerived;
    typedef ConstraintXd Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef Motion Bias_t;

    typedef Eigen::Matrix<double,6,Eigen::Dynamic> F_t;
    // [ABA]
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> U_t;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> D_t;
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> UD_t;

    typedef Eigen::Matrix<double,Eigen::Dynamic,1> ConfigVector_t;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1> TangentVector_t;
  };
  template<> struct traits<JointData> { typedef Joint JointDerived; };
  template<> struct traits<JointModel> { typedef Joint JointDerived; };

  struct JointData : public JointDataBase<JointData> , JointDataVariant
  {
    typedef JointDataVariant JointDataBoostVariant;

    typedef Joint JointDerived;
    SE3_JOINT_TYPEDEF;

    JointDataVariant& toVariant() { return *static_cast<JointDataVariant*>(this); }
    const JointDataVariant& toVariant() const { return *static_cast<const JointDataVariant*>(this); }

    const Constraint_t      S() const  { return constraint_xd(*this); }
    const Transformation_t  M() const  { return joint_transform(*this); } // featherstone book, page 78 (sXp)
    const Motion_t          v() const  { return motion(*this); }
    const Bias_t            c() const  { return bias(*this); }
    
    // // [ABA CCRBA]
    const U_t               U()     const { return u_inertia(*this); }
    U_t                     U()           { return u_inertia(*this); }
    const D_t               Dinv()  const { return dinv_inertia(*this); }
    const UD_t              UDinv() const { return udinv_inertia(*this); }


    JointData() : JointDataBoostVariant() {}
    JointData(const JointDataVariant & jdata ) : JointDataBoostVariant(jdata){}

  };

  struct JointModel : public JointModelBase<JointModel> , JointModelVariant
  {
    typedef JointModelVariant JointModelBoostVariant;

    typedef Joint JointDerived;
    SE3_JOINT_TYPEDEF;
    SE3_JOINT_USE_INDEXES;
    using JointModelBase<JointModel>::id;
    using JointModelBase<JointModel>::setIndexes;

    JointModelVariant& toVariant() { return *static_cast<JointModelVariant*>(this); }
    const JointModelVariant& toVariant() const { return *static_cast<const JointModelVariant*>(this); }

    JointDataVariant createData() 
    {
      return ::se3::createData(*this);
    }


    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      calc_zero_order(*this, data, qs);

    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      calc_first_order(*this, data, qs, vs);
    }
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      ::se3::calc_aba(*this, data, I, update_I);
    }

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      return ::se3::integrate(*this, qs, vs);
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    {
      return ::se3::interpolate(*this, q0, q1, u);
    }

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & lower_pos_limit, const ConfigVector_t & upper_pos_limit ) const throw (std::runtime_error)
    { 
      return ::se3::randomConfiguration(*this, lower_pos_limit, upper_pos_limit);
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return ::se3::difference(*this, q0, q1);
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return ::se3::distance(*this, q0, q1);
    }

    ConfigVector_t neutralConfiguration_impl() const
    { 
      return ::se3::neutralConfiguration(*this);
    } 

    JointModel() : JointModelBoostVariant() {}
    JointModel( const JointModelVariant & model_variant ) : JointModelBoostVariant(model_variant)
    {}


    const std::string shortname()
    {
      return ::se3::shortname(*this);
    }

    JointModel& operator=( const JointModel& other) 
    {
      *this = other;
      return *this;
    }

    JointModel& operator=( const JointModelVariant& other)
    {
      *this = other;
      return *this;
    }

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModel> & jmodel) const
    {
      return jmodel.id() == id()
          && jmodel.idx_q() == idx_q()
          && jmodel.idx_v() == idx_v();
    }


    int     nq_impl() const { return ::se3::nq(*this); }
    int     nv_impl() const { return ::se3::nv(*this); }

    int     idx_q()   const { return ::se3::idx_q(*this); }
    int     idx_v()   const { return ::se3::idx_v(*this); }

    JointIndex     id()      const { return ::se3::id(*this); }

    // void setIndexes(JointIndex ,int ,int ) { SE3_STATIC_ASSERT(false, THIS_METHOD_SHOULD_NOT_BE_CALLED_ON_DERIVED_CLASS); }
    void setIndexes(JointIndex id,int nq,int nv) 
    {
      ::se3::setIndexes(*this,id, nq, nv);
    }
  };
  
  typedef std::vector<JointData> JointDataVector;  
  typedef std::vector<JointModel> JointModelVector;

} // namespace se3


#endif // ifndef __se3_joint_model_hpp__
