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
			    const int&,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(RneaForwardStep);

    template<typename JointModel>
    static int algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const int &i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v,
		    const Eigen::VectorXd & a)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[(Model::Index)i];
      data.liMi[(Model::Index)i] = model.jointPlacements[(Model::Index)i]*jdata.M();
      
      data.v[(Model::Index)i] = jdata.v();
      if(parent>0) data.v[(Model::Index)i] += data.liMi[(Model::Index)i].actInv(data.v[parent]);
      
      data.a[(Model::Index)i]  = jdata.S()*jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[(Model::Index)i] ^ jdata.v()) ; 
      data.a[(Model::Index)i] += data.liMi[(Model::Index)i].actInv(data.a[parent]);
      
      data.f[(Model::Index)i] = model.inertias[(Model::Index)i]*data.a[(Model::Index)i] + model.inertias[(Model::Index)i].vxiv(data.v[(Model::Index)i]); // -f_ext
      return 0;
    }

  };

  struct RneaBackwardStep : public fusion::JointVisitor<RneaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&,
				  const int &>  ArgsType;
    
    JOINT_VISITOR_INIT(RneaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data,
		     int i)
    {
      const Model::Index & parent  = model.parents[(Model::Index)i];      
      jmodel.jointVelocitySelector(data.tau)  = jdata.S().transpose()*data.f[(Model::Index)i];
      if(parent>0) data.f[(Model::Index)parent] += data.liMi[(Model::Index)i].act(data.f[(Model::Index)i]);
    }
  };

  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a)
  {
    data.v[0] = Motion::Zero();
    data.a[0] = -model.gravity;

    for( int i=1;i<model.nbody;++i )
      {
	RneaForwardStep::run(model.joints[(Model::Index)i],data.joints[(Model::Index)i],
			     RneaForwardStep::ArgsType(model,data,i,q,v,a));
      }
    
    for( int i=model.nbody-1;i>0;--i )
      {
	RneaBackwardStep::run(model.joints[(Model::Index)i],data.joints[(Model::Index)i],
	 		      RneaBackwardStep::ArgsType(model,data,i));
      }

    return data.tau;
  }
} // namespace se3

#endif // ifndef __se3_rnea_hpp__

