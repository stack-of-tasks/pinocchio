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

#ifndef __se3_kinematics_hpp__
#define __se3_kinematics_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline void geometry(const Model & model,
                       Data & data,
                       const Eigen::VectorXd & q);

  inline void kinematics(const Model & model,
			 Data & data,
			 const Eigen::VectorXd & q,
			 const Eigen::VectorXd & v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct GeometryStep : public fusion::JointVisitor<GeometryStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Model::Index,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (GeometryStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::Index i,
                     const Eigen::VectorXd & q)
    {
      using namespace se3;

      jmodel.calc (jdata.derived (), q);

      const Model::Index & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M ();

      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
    
  };

  inline void
  geometry(const Model & model,
           Data & data,
           const Eigen::VectorXd & q)
  {
    for (Model::Index i=1; i < (Model::Index) model.nbody; ++i)
    {
      GeometryStep::run(model.joints[i],
                        data.joints[i],
                        GeometryStep::ArgsType (model,data,i,q)
                        );
    }
  }

  struct KinematicsStep : public fusion::JointVisitor<KinematicsStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Model::Index,
				   const Eigen::VectorXd &,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(KinematicsStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const Model::Index i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[i];
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
    }

  };

  inline void
  kinematics(const Model & model, Data& data,
	     const Eigen::VectorXd & q,
	     const Eigen::VectorXd & v)
  {
    data.v[0] = Motion::Zero();

    for( Model::Index i=1; i<(Model::Index) model.nbody; ++i )
      {
	KinematicsStep::run(model.joints[i],data.joints[i],
			    KinematicsStep::ArgsType(model,data,i,q,v));
      }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hpp__

