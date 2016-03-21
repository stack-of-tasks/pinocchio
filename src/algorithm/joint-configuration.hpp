//
// Copyright (c) 2016 CNRS
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

  /**
   * @brief      Integrate the model for a constant derivative during unit time
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Velocity (size model.nv)
   * @param      result  Resulting configuration result (size model.nq)
   */
  inline void integrate(const Model & model,
                             Data & data,
                             const Eigen::VectorXd & q,
                             const Eigen::VectorXd & v,
                             Eigen::VectorXd & result);


  /**
   * @brief      Interpolate the model between two configurations
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q1      Initial configuration vector (size model.nq)
   * @param[in]  q2      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   * @param      result  The interpolated configuration (q1 if u = 0, q2 if u = 1)
   */
  inline void interpolate(const Model & model,
                               Data & data,
                               const Eigen::VectorXd & q1,
                               const Eigen::VectorXd & q2,
                               const double u,
                               Eigen::VectorXd & result);


  /**
   * @brief      the constant derivative that must be integrated during unit time to go from q2 to q1
   *
   * @param[in]  model   Model to be differentiated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q1      Wished configuration (size model.nq)
   * @param[in]  q2      Initial configuration (size model.nq)
   * @param      result  The corresponding velocity (size model.nv)
   */
  inline void differentiate(const Model & model,
                                 Data & data,
                                 const Eigen::VectorXd & q1,
                                 const Eigen::VectorXd & q2,
                                 Eigen::VectorXd & result);


  /**
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  data       Corresponding Data to the Model
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @param[in]  q2         Configuration 2 (size model.nq)
   * @param      distances  The corresponding distances for each joint (size model.nbody-1 = number of joints)
   */
  inline void distance(const Model & model,
                            Data & data,
                            const Eigen::VectorXd & q1,
                            const Eigen::VectorXd & q2,
                            Eigen::VectorXd & distances);


  /**
   * @brief      Generate a random configuration vector.
   *
   * @param[in]  model   Model we want to generate a configuration vector of
   * @param[in]  data    Corresponding Data to the Model
   * @param      config  The resulted configuration vector (size model.nq)
   */
  inline void random(const Model & model,
                          Data & data,
                          Eigen::VectorXd & config);
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct IntegrateStep : public fusion::JointVisitor<IntegrateStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(IntegrateStep);

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
  integrate(const Model & model,
                 Data & data,
                 const Eigen::VectorXd & q,
                 const Eigen::VectorXd & v,
                 Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      IntegrateStep::run(model.joints[i],
                              data.joints[i],
                              IntegrateStep::ArgsType (model, data, q, v, result)
                              );
    }
  }


  struct InterpolateStep : public fusion::JointVisitor<InterpolateStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  const double,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(InterpolateStep);

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
  interpolate(const Model & model,
                   Data & data,
                   const Eigen::VectorXd & q1,
                   const Eigen::VectorXd & q2,
                   const double u,
                   Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      InterpolateStep::run(model.joints[i],
                              data.joints[i],
                              InterpolateStep::ArgsType (model, data, q1, q2, u, result)
                              );
    }
  }

  struct DifferentiateStep : public fusion::JointVisitor<DifferentiateStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(DifferentiateStep);

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
  differentiate(const Model & model,
                     Data & data,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2,
                     Eigen::VectorXd & result)
  {

    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      DifferentiateStep::run(model.joints[i],
                              data.joints[i],
                              DifferentiateStep::ArgsType (model, data, q1, q2, result)
                              );
    }
  }

  struct DistanceStep : public fusion::JointVisitor<DistanceStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Model::JointIndex,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(DistanceStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model &,
                     se3::Data &,
                     const Model::JointIndex i,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2,
                     Eigen::VectorXd & distances) 
    {
      using namespace Eigen;
      using namespace se3;
      
      distances[(long)i] = jmodel.distance(q1, q2);
    }

  };

  inline void
  distance(const Model & model,
               Data & data,
               const Eigen::VectorXd & q1,
               const Eigen::VectorXd & q2,
               Eigen::VectorXd & distances)
  {
    assert(distances.size() == model.nbody-1);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      DistanceStep::run(model.joints[i],
                              data.joints[i],
                              DistanceStep::ArgsType (model, data, i-1, q1, q2, distances)
                              );
    }
  }

  struct RandomStep : public fusion::JointVisitor<RandomStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT(RandomStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> &,
                     const se3::Model & model,
                     se3::Data &,
                     Eigen::VectorXd & config) 
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.jointConfigSelector(config) = jmodel.random(jmodel.jointConfigSelector(model.lowerPositionLimit),
                                                          jmodel.jointConfigSelector(model.upperPositionLimit)
                                                          );
    }

  };

  inline void
  random(const Model & model,
               Data & data,
               Eigen::VectorXd & config)
  {
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      RandomStep::run(model.joints[i],
                           data.joints[i],
                           RandomStep::ArgsType (model, data, config)
                           );
    }
  }
} // namespace se3

#endif // ifndef __se3_joint_configuration_hpp__

