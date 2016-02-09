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

#ifndef __se3_rnea_hpp__
#define __se3_rnea_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct RneaForwardStep : public fusion::JointVisitor<RneaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
			    se3::Data&,
			    const Model::Index,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(RneaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const Model::Index i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v,
		    const Eigen::VectorXd & a)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      data.a_gf[i] = jdata.S()*jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };

  struct RneaBackwardStep : public fusion::JointVisitor<RneaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&,
				  const Model::Index>  ArgsType;
    
    JOINT_VISITOR_INIT(RneaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data,
		     Model::Index i)
    {
      const Model::JointIndex & parent  = model.parents[i];      
      jmodel.jointVelocitySelector(data.tau)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };

  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a)
  {
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;

    for( Model::JointIndex i=1;i<(Model::JointIndex)model.nbody;++i )
      {
	RneaForwardStep::run(model.joints[i],data.joints[i],
			     RneaForwardStep::ArgsType(model,data,i,q,v,a));
      }
    
    for( Model::JointIndex i=(Model::JointIndex)model.nbody-1;i>0;--i )
      {
	RneaBackwardStep::run(model.joints[i],data.joints[i],
	 		      RneaBackwardStep::ArgsType(model,data,i));
      }

    return data.tau;
  }
} // namespace se3

#endif // ifndef __se3_rnea_hpp__

