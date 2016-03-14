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

#ifndef __se3_joint_configuration_hpp__
#define __se3_joint_configuration_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline void integrateModel(const Model & model,
                             Data & data,
                             const Eigen::VectorXd & q,
                             const Eigen::VectorXd & v,
                             Eigen::VectorXd & result);

  inline void interpolateModel(const Model & model,
                               Data & data,
                               const Eigen::VectorXd & q1,
                               const Eigen::VectorXd & q2,
                               const double u,
                               Eigen::VectorXd & result);

  inline void differentiateModel(const Model & model,
                                 Data & data,
                                 const Eigen::VectorXd & q1,
                                 const Eigen::VectorXd & q2,
                                 Eigen::VectorXd & result);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct IntegrateModelStep : public fusion::JointVisitor<IntegrateModelStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(IntegrateModelStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data &,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     Eigen::VectorXd & result) 
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.jointConfigSelector(result) = jmodel.integrate(q, v); // if computation needed, do it here, or may be in lowerPosLimit
    }

  };

  inline void
  integrateModel(const Model & model,
                 Data & data,
                 const Eigen::VectorXd & q,
                 const Eigen::VectorXd & v,
                 Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      IntegrateModelStep::run(model.joints[i],
                              data.joints[i],
                              IntegrateModelStep::ArgsType (model, data, q, v, result)
                              );
    }
  }


  struct InterpolateModelStep : public fusion::JointVisitor<InterpolateModelStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  const double,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(InterpolateModelStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data &,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2,
                     const double u,
                     Eigen::VectorXd & result) 
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.jointConfigSelector(result) = jmodel.interpolate(q1, q2, u);
    }

  };

  inline void
  interpolateModel(const Model & model,
                   Data & data,
                   const Eigen::VectorXd & q1,
                   const Eigen::VectorXd & q2,
                   const double u,
                   Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      InterpolateModelStep::run(model.joints[i],
                              data.joints[i],
                              InterpolateModelStep::ArgsType (model, data, q1, q2, u, result)
                              );
    }
  }

  struct DifferentiateModelStep : public fusion::JointVisitor<DifferentiateModelStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(DifferentiateModelStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data &,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2,
                     Eigen::VectorXd & result) 
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.jointVelocitySelector(result) = jmodel.difference(q1, q2);
    }

  };

  inline void
  differentiateModel(const Model & model,
                     Data & data,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2,
                     Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      DifferentiateModelStep::run(model.joints[i],
                              data.joints[i],
                              DifferentiateModelStep::ArgsType (model, data, q1, q2, result)
                              );
    }
  }
} // namespace se3

#endif // ifndef __se3_joint_configuration_hpp__

