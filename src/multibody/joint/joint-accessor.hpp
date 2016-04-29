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

#ifndef __se3_joint_accessor_hpp__
#define __se3_joint_accessor_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"

namespace se3
{

  struct JointAccessor;
  struct JointModelAccessor;
  struct JointDataAccessor;

  template<>
  struct traits<JointAccessor>
  {

    enum {
      NQ = -1, // Dynamic because unknown at compilation
      NV = -1
    };
    typedef JointDataAccessor JointData;
    typedef JointModelAccessor JointModel;
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
  template<> struct traits<JointDataAccessor> { typedef JointAccessor Joint; };
  template<> struct traits<JointModelAccessor> { typedef JointAccessor Joint; };

  struct JointDataAccessor : public JointDataBase<JointDataAccessor> 
  {
    typedef JointAccessor Joint;
    SE3_JOINT_TYPEDEF;


    JointDataVariant & j_data_variant;

    const Constraint_t      S() const  { return constraint_xd(j_data_variant); }
    const Transformation_t  M() const  { return joint_transform(j_data_variant); } // featherstone book, page 78 (sXp)
    const Motion_t          v() const  { return motion(j_data_variant); }
    const Bias_t            c() const  { return bias(j_data_variant); }
    
    // // [ABA CCRBA]
    const U_t               U()     const { return u_inertia(j_data_variant); }
    U_t                     U()           { return u_inertia(j_data_variant); }
    const D_t               Dinv()  const { return dinv_inertia(j_data_variant); }
    const UD_t              UDinv() const { return udinv_inertia(j_data_variant); }


    // JointDataAccessor()  {} // can become necessary if we want a vector of jointDataAccessor ?
    JointDataAccessor( JointDataVariant & jdata ) : j_data_variant(jdata){}

  };

  struct JointModelAccessor : public JointModelBase<JointModelAccessor> 
  {
    typedef JointAccessor Joint;
    SE3_JOINT_TYPEDEF;
    SE3_JOINT_USE_INDEXES;
    using JointModelBase<JointModelAccessor>::id;
    using JointModelBase<JointModelAccessor>::setIndexes;

    const JointModelVariant & j_model_variant;

    JointDataVariant createData() 
    {
      return ::se3::createData(j_model_variant);
    }


    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      calc_zero_order(j_model_variant, data.j_data_variant, qs);

    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      calc_first_order(j_model_variant, data.j_data_variant, qs, vs);
    }
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      ::se3::calc_aba(j_model_variant, data.j_data_variant, I, update_I);
    }

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      return ::se3::integrate(j_model_variant, qs, vs);
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    {
      return ::se3::interpolate(j_model_variant, q0, q1, u);
    }

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & lower_pos_limit, const ConfigVector_t & upper_pos_limit ) const throw (std::runtime_error)
    { 
      return ::se3::randomConfiguration(j_model_variant, lower_pos_limit, upper_pos_limit);
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return ::se3::difference(j_model_variant, q0, q1);
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return ::se3::distance(j_model_variant, q0, q1);
    }

    // JointModelAccessor() : j_model_variant() {} // can become necessary if we want a vector of jointModelAccessor ?
    JointModelAccessor( const JointModelVariant & model_variant ) : j_model_variant(model_variant)
    {}

    int     nq_impl() const { return ::se3::nq(j_model_variant); }
    int     nv_impl() const { return ::se3::nv(j_model_variant); }

    int     idx_q()   const { return ::se3::idx_q(j_model_variant); }
    int     idx_v()   const { return ::se3::idx_v(j_model_variant); }

    JointIndex     id()      const { return ::se3::id(j_model_variant); }

    void setIndexes(JointIndex ,int ,int ) { SE3_STATIC_ASSERT(false, THIS_METHOD_SHOULD_NOT_BE_CALLED_ON_DERIVED_CLASS); }
  };
  

} // namespace se3


#endif // ifndef __se3_joint_accessor_hpp__
