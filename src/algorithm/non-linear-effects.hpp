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

#ifndef __se3_non_linear_effects_hpp__
#define __se3_non_linear_effects_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline const Eigen::VectorXd &
  nonLinearEffects(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct NLEForwardStep : public fusion::JointVisitor<NLEForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
			    se3::Data &,
			    const size_t,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(NLEForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model & model,
		    se3::Data & data,
		    const size_t i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[(size_t) parent]);
      
      data.a_gf[i]  = jdata.c() + (data.v[i] ^ jdata.v());
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[(size_t) parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };

  struct NLEBackwardStep : public fusion::JointVisitor<NLEBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
      Data &,
      const size_t &>  ArgsType;
    
    JOINT_VISITOR_INIT(NLEBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model & model,
		     Data & data,
		     const size_t i)
    {
      const Model::JointIndex & parent  = model.parents[i];      
      jmodel.jointVelocitySelector(data.nle)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[(size_t) parent] += data.liMi[i].act(data.f[i]);
    }
  };

  inline const Eigen::VectorXd &
  nonLinearEffects(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v)
  {
    data.v[0].setZero ();
    data.a_gf[0] = -model.gravity;

    for( size_t i=1;i<(size_t) model.nbody;++i )
      {
        NLEForwardStep::run(model.joints[i],data.joints[i],
                            NLEForwardStep::ArgsType(model,data,i,q,v));
      }

    for( size_t i=(size_t) (model.nbody-1);i>0;--i )
    {
      NLEBackwardStep::run(model.joints[i],data.joints[i],
                           NLEBackwardStep::ArgsType(model,data,i));
    }
    
    return data.nle;
  }
} // namespace se3

#endif // ifndef __se3_non_linear_effects_hpp__

