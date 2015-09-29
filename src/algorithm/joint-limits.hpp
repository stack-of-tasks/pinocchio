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

#ifndef __se3_joint_limits_hpp__
#define __se3_joint_limits_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline void jointLimits(const Model & model,
                          Data & data,
                          const Eigen::VectorXd & q);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct JointLimitsStep : public fusion::JointVisitor<JointLimitsStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(JointLimitsStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data & data,
                     const Eigen::VectorXd &) // TODO: make joint limits depend explicitely on the current state (q,v)
    {
      using namespace Eigen;
      using namespace se3;
      
      // TODO: as limits are static, no need of q, nor computations
      jmodel.jointVelocitySelector(data.effortLimit) = jmodel.maxEffortLimit();
      jmodel.jointVelocitySelector(data.velocityLimit) = jmodel.maxVelocityLimit();

      jmodel.jointConfigSelector(data.lowerPositionLimit) = jmodel.lowerPosLimit(); // if computation needed, do it here, or may be in lowerPosLimit
      jmodel.jointConfigSelector(data.upperPositionLimit) = jmodel.upperPosLimit();
    }

  };

  inline void
  jointLimits(const Model & model,
              Data& data,
              const Eigen::VectorXd & q)
  {

    for( Model::Index i=1; i<(Model::Index) model.nbody; ++i )
    {
      JointLimitsStep::run(model.joints[i],
                        data.joints[i],
                        JointLimitsStep::ArgsType (model,data,q)
                        );
    }
  }
} // namespace se3

#endif // ifndef __se3_joint_limits_hpp__

