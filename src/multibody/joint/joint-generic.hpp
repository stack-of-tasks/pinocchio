//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_joint_generic_hpp__
#define __se3_joint_generic_hpp__

#include "pinocchio/exception.hpp"
#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/joint/joint-variant.hpp"

namespace se3
{

  struct JointGeneric;
  struct JointModelGeneric;
  struct JointDataGeneric;

  template<>
  struct traits<JointGeneric>
  {
    typedef JointDataGeneric JointData;
    typedef JointModelGeneric JointModel;
    typedef ConstraintXd Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef Motion Bias_t;
    enum {
      NQ = -1,
      NV = -1
    };
  };
  template<> struct traits<JointDataGeneric> { typedef JointGeneric JointDerived; };
  template<> struct traits<JointModelGeneric> { typedef JointGeneric JointDerived; };

  struct JointDataGeneric : public JointDataBase<JointDataGeneric> 
  {
    typedef JointGeneric JointDerived;
    SE3_JOINT_TYPEDEF;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    JointDataGeneric()  {}
    JointDataGeneric( const JointDataVariant & jvar );
  };

  struct JointModelGeneric : public JointModelBase<JointModelGeneric> 
  {
    typedef JointGeneric JointDerived;
    SE3_JOINT_TYPEDEF;
    SE3_JOINT_USE_INDEXES;

    JointData createData() const { return JointData(); }

    virtual void jcalc(JointData& /*data*/, 
		       const Eigen::VectorXd & /*qs*/) const 
    {
      throw se3::Exception("Virtual function calc not implemented in the generic class. Derive it.");
    }
    virtual void jcalc(JointData& /*data*/, 
		       const Eigen::VectorXd & /*qs*/, 
		       const Eigen::VectorXd & /*vs*/ ) const 
    {
      throw se3::Exception("Virtual function calc not implemented in the generic class. Derive it.");
    }

    JointModelGeneric() {}
    JointModelGeneric( const JointModelVariant & jvar );
    JointModelGeneric & operator= (const JointModelGeneric& clone)
    { 
      setIndexes(clone.idx_q(),clone.idx_v()); return *this;
    }
  };

  /* --- Convert Data ------------------------------------------------------ */
  namespace internal
  {
    struct JointDataVariantToGeneric : public boost::static_visitor< JointDataGeneric >
    {
      template <typename D>
      JointDataGeneric operator() ( const JointDataBase<D> & jdata ) const
      {
	JointDataGeneric jgen;
	jgen.S = jdata.S();
	jgen.M = jdata.M();
	jgen.v = jdata.v();
	jgen.c = jdata.c();
	return jgen;
      }

      static JointDataGeneric run ( const JointDataVariant & jdata )
      {
	return boost::apply_visitor( JointDataVariantToGeneric(),jdata );
      }
    };
  }// namespace internal

  JointDataGeneric::JointDataGeneric( const JointDataVariant & jvar )
  {
    const JointDataGeneric & clone = internal::JointDataVariantToGeneric::run(jvar);
    (*this) = clone;
  }
  
  /* --- Convert Model ----------------------------------------------------- */
  namespace internal
  {
    struct JointModelVariantToGeneric : public boost::static_visitor< JointModelGeneric >
    {
      template <typename D>
      JointModelGeneric operator() ( const JointModelBase<D> & jmodel ) const
      {
	JointModelGeneric jgen;
	jgen.setIndexes(jmodel.idx_q(),jmodel.idx_v());
	return jgen;
      }

      static JointModelGeneric run ( const JointModelVariant & jmodel )
      {
	return boost::apply_visitor( JointModelVariantToGeneric(),jmodel );
      }
    };
  }// namespace internal

  JointModelGeneric::JointModelGeneric( const JointModelVariant & jvar )
  {
    const JointModelGeneric & clone = internal::JointModelVariantToGeneric::run(jvar);
    (*this) = clone;
  }

} // namespace se3


#endif // ifndef __se3_joint_generic_hpp__
