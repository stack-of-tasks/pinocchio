//
// Copyright (c) 2015-2016 CNRS
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
    ///
    /// \brief Browse through the model tree structure with an empty step
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    ///
  inline void emptyForwardPass(const Model & model,
                               Data & data);
  
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q);

  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v);
  
  inline void forwardKinematics(const Model & model,
                                Data & data,
                                const Eigen::VectorXd & q,
                                const Eigen::VectorXd & v,
                                const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  
  struct emptyForwardStep : public fusion::JointVisitor<emptyForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &
                                  > ArgsType;
    
    JOINT_VISITOR_INIT (emptyForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> &,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data &)
    { // do nothing
    }
    
  };
  
  inline void emptyForwardPass(const Model & model,
                               Data & data)
  {
    for (Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i)
    {
      emptyForwardStep::run(model.joints[i],
                            data.joints[i],
                            emptyForwardStep::ArgsType (model,data)
                            );
    }
  }
  
  struct ForwardKinematicZeroStep : public fusion::JointVisitor<ForwardKinematicZeroStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Model::JointIndex,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (ForwardKinematicZeroStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::JointIndex i,
                     const Eigen::VectorXd & q)
    {
      using namespace se3;

      jmodel.calc (jdata.derived (), q);

      const Model::JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M ();

      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
    
  };

  inline void
  forwardKinematics(const Model & model,
                    Data & data,
                    const Eigen::VectorXd & q)
  {
    for (Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i)
    {
      ForwardKinematicZeroStep::run(model.joints[i],
                        data.joints[i],
                        ForwardKinematicZeroStep::ArgsType (model,data,i,q)
                        );
    }
  }

  struct ForwardKinematicFirstStep : public fusion::JointVisitor<ForwardKinematicFirstStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Model::JointIndex,
				   const Eigen::VectorXd &,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(ForwardKinematicFirstStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const Model::JointIndex i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::JointIndex & parent = model.parents[i];
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
  forwardKinematics(const Model & model, Data & data,
                    const Eigen::VectorXd & q,
                    const Eigen::VectorXd & v)
  {
    data.v[0].setZero();

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
      {
	ForwardKinematicFirstStep::run(model.joints[i],data.joints[i],
			    ForwardKinematicFirstStep::ArgsType(model,data,i,q,v));
      }
  }
  
  struct ForwardKinematicSecondStep : public fusion::JointVisitor<ForwardKinematicSecondStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
    se3::Data&,
    const Model::JointIndex,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(ForwardKinematicSecondStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::JointIndex i,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a)
    {
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::JointIndex & parent = model.parents[i];
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      data.a[i]  = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a[i] += data.liMi[i].actInv(data.a[parent]);
    }
  };
  
  inline void
  forwardKinematics(const Model & model, Data & data,
                    const Eigen::VectorXd & q,
                    const Eigen::VectorXd & v,
                    const Eigen::VectorXd & a)
  {
    data.v[0].setZero();
    data.a[0].setZero();
    
    for( Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i )
    {
      ForwardKinematicSecondStep::run(model.joints[i],data.joints[i],
                        ForwardKinematicSecondStep::ArgsType(model,data,i,q,v,a));
    }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hpp__

