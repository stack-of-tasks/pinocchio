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

#ifndef __se3_joint_variant_hpp__
#define __se3_joint_variant_hpp__

#include "pinocchio/multibody/joint.hpp"
#include <Eigen/StdVector>
#include <boost/variant.hpp>

namespace se3
{
  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned, JointModelSpherical, JointModelSphericalZYX, JointModelPX, JointModelPY, JointModelPZ, JointModelPrismaticUnaligned, JointModelFreeFlyer, JointModelPlanar, JointModelTranslation, JointModelDense<-1,-1> > JointModelVariant;
  typedef boost::variant< JointDataRX, JointDataRY, JointDataRZ, JointDataRevoluteUnaligned, JointDataSpherical, JointDataSphericalZYX, JointDataPX, JointDataPY, JointDataPZ, JointDataPrismaticUnaligned, JointDataFreeFlyer, JointDataPlanar, JointDataTranslation, JointDataDense<-1,-1> > JointDataVariant;

  typedef std::vector<JointModelVariant> JointModelVector;
  typedef std::vector<JointDataVariant> JointDataVector;

  class CreateJointData: public boost::static_visitor<JointDataVariant>
  {
  public:
    template<typename D>
    JointDataVariant operator()(const JointModelBase<D> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( CreateJointData(), jmodel ); }
  };

  class Joint_nv: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & ) const
    { return D::NV; }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_nv(), jmodel ); }
  };
  inline int nv(const JointModelVariant & jmodel) { return Joint_nv::run(jmodel); }

  class Joint_nq: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & ) const
    { return D::NQ; }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_nq(), jmodel ); }
  };
  inline int nq(const JointModelVariant & jmodel) { return Joint_nq::run(jmodel); }

  class Joint_idx_q: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_q(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_idx_q(), jmodel ); }
  };
  inline int idx_q(const JointModelVariant & jmodel) { return Joint_idx_q::run(jmodel); }

  class Joint_idx_v: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_v(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( Joint_idx_v(), jmodel ); }
  };
  inline int idx_v(const JointModelVariant & jmodel) { return Joint_idx_v::run(jmodel); }

  // template <class Constraint>
  class Joint_constraint: public boost::static_visitor< ConstraintXd >
  {
  public:
    template <typename D>
    ConstraintXd operator()(const JointDataBase<D> & jdata) const
    { return ConstraintXd(jdata.S()); }
    
    static ConstraintXd run( const JointDataVariant & jdata)
    { return boost::apply_visitor( Joint_constraint (), jdata ); }
  };
  inline ConstraintXd constraint_xd(const JointDataVariant & jdata) { return Joint_constraint::run(jdata); }

} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointModelVariant)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointDataVariant)

#endif // ifndef __se3_joint_variant_hpp__
