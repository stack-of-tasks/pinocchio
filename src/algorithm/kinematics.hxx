//
// Copyright (c) 2016-2017 CNRS
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

#ifndef __se3_kinematics_hxx__
#define __se3_kinematics_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

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
                     se3::JointDataBase<typename JointModel::JointDataDerived> &,
                     const se3::Model &,
                     se3::Data &)
    { // do nothing
    }
    
  };
  
  inline void emptyForwardPass(const Model & model,
                               Data & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for (Model::JointIndex i=1; i < (Model::JointIndex) model.njoints; ++i)
    {
      emptyForwardStep::run(model.joints[i],
                            data.joints[i],
                            emptyForwardStep::ArgsType (model,data)
                            );
    }
  }
  
  inline void updateGlobalPlacements(const Model & model, Data & data)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    for (Model::JointIndex i=1; i < (Model::JointIndex) model.njoints; ++i)
    {
      const Model::JointIndex & parent = model.parents[i];
      
      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
  }
  
  struct ForwardKinematicZeroStep : public fusion::JointVisitor<ForwardKinematicZeroStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (ForwardKinematicZeroStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];

      jmodel.calc (jdata.derived (), q);

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
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    for (Model::JointIndex i=1; i < (Model::JointIndex) model.njoints; ++i)
    {
      ForwardKinematicZeroStep::run(model.joints[i], data.joints[i],
                                    ForwardKinematicZeroStep::ArgsType (model,data,q)
                                    );
    }
  }

  struct ForwardKinematicFirstStep : public fusion::JointVisitor<ForwardKinematicFirstStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
				   se3::Data &,
				   const Eigen::VectorXd &,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(ForwardKinematicFirstStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
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
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      ForwardKinematicFirstStep::run(model.joints[i],data.joints[i],
                                     ForwardKinematicFirstStep::ArgsType(model,data,q,v));
    }
  }
  
  struct ForwardKinematicSecondStep : public fusion::JointVisitor<ForwardKinematicSecondStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(ForwardKinematicSecondStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);

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
    assert(q.size() == model.nq && "The configuration vector is not of right size");
    assert(v.size() == model.nv && "The velocity vector is not of right size");
    assert(a.size() == model.nv && "The acceleration vector is not of right size");
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a[0].setZero();
    
    for( Model::JointIndex i=1; i < (Model::JointIndex) model.njoints; ++i )
    {
      ForwardKinematicSecondStep::run(model.joints[i],data.joints[i],
                                      ForwardKinematicSecondStep::ArgsType(model,data,q,v,a));
    }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hxx__
