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
   * @brief      Integrate a configuration for the specified model for a constant derivative during one unit time
   *
   * @param[in]  model   Model that must be integrated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q       Initial configuration (size model.nq)
   * @param[in]  v       Velocity (size model.nv)
   * @return     The integrated configuration (size model.nq)
   */
  inline Eigen::VectorXd integrate(const Model & model,
                             const Eigen::VectorXd & q,
                             const Eigen::VectorXd & v);


  /**
   * @brief      Interpolate the model between two configurations
   *
   * @param[in]  model   Model to be interpolated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q0      Initial configuration vector (size model.nq)
   * @param[in]  q1      Final configuration vector (size model.nq)
   * @param[in]  u       u in [0;1] position along the interpolation.
   * @return     The interpolated configuration (q0 if u = 0, q1 if u = 1)
   */
  inline Eigen::VectorXd interpolate(const Model & model,
                               const Eigen::VectorXd & q0,
                               const Eigen::VectorXd & q1,
                               const double u);


  /**
   * @brief      the constant derivative that must be integrated during unit time to go from q0 to q1
   *
   * @param[in]  model   Model to be differentiated
   * @param[in]  data    Corresponding Data to the Model
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   * @return     The corresponding velocity (size model.nv)
   */
  inline Eigen::VectorXd differentiate(const Model & model,
                                 const Eigen::VectorXd & q0,
                                 const Eigen::VectorXd & q1);


  /**
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  data       Corresponding Data to the Model
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @return     The corresponding distances for each joint (size model.nbody-1 = number of joints)
   */
  inline Eigen::VectorXd distance(const Model & model,
                            const Eigen::VectorXd & q0,
                            const Eigen::VectorXd & q1);


  /**
   * @brief      Generate a random configuration vector uniformly sampled among joint limits
   *
   * @param[in]  model   Model we want to generate a configuration vector of
   * @param[in]  data    Corresponding Data to the Model
   * @return     The resulted configuration vector (size model.nq)
   */
  inline Eigen::VectorXd random(const Model & model);

  /**
   * @brief      Generate a random configuration vector.
   *
   * @param[in]  model   Model we want to generate a configuration vector of
   * @param[in]  data    Corresponding Data to the Model
   * @return     The resulted configuration vector (size model.nq)
   */
  inline Eigen::VectorXd uniformlySample(const Model & model);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct IntegrateStep : public fusion::JointModelVisitor<IntegrateStep>
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(IntegrateStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     Eigen::VectorXd & result) 
    {
     
      jmodel.jointConfigSelector(result) = jmodel.integrate(q, v); // if computation needed, do it here, or may be in lowerPosLimit
    }

  };

  inline Eigen::VectorXd
  integrate(const Model & model,
                 const Eigen::VectorXd & q,
                 const Eigen::VectorXd & v)
  {
    Eigen::VectorXd integ(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      IntegrateStep::run(model.joints[i],
                          IntegrateStep::ArgsType (q, v, integ)
                          );
    }
    return integ;
  }


  struct InterpolateStep : public fusion::JointModelVisitor<InterpolateStep>
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  const double,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(InterpolateStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1,
                     const double u,
                     Eigen::VectorXd & result) 
    {
      jmodel.jointConfigSelector(result) = jmodel.interpolate(q0, q1, u);
    }

  };

  inline Eigen::VectorXd
  interpolate(const Model & model,
               const Eigen::VectorXd & q0,
               const Eigen::VectorXd & q1,
               const double u)
  {
    Eigen::VectorXd interp(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      InterpolateStep::run(model.joints[i],
                            InterpolateStep::ArgsType (q0, q1, u, interp)
                            );
    }
    return interp;
  }

  struct DifferentiateStep : public fusion::JointModelVisitor<DifferentiateStep>
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(DifferentiateStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1,
                     Eigen::VectorXd & result) 
    {
      jmodel.jointVelocitySelector(result) = jmodel.difference(q0, q1);
    }

  };

  inline Eigen::VectorXd
  differentiate(const Model & model,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1)
  {
    Eigen::VectorXd diff(model.nv);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      DifferentiateStep::run(model.joints[i],
                              DifferentiateStep::ArgsType (q0, q1, diff)
                              );
    }
    return diff;
  }

  struct DistanceStep : public fusion::JointModelVisitor<DistanceStep>
  {
    typedef boost::fusion::vector<const Model::JointIndex,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(DistanceStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const Model::JointIndex i,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1,
                     Eigen::VectorXd & distances) 
    {
      distances[(long)i] = jmodel.distance(q0, q1);
    }

  };

  inline Eigen::VectorXd
  distance(const Model & model,
               const Eigen::VectorXd & q0,
               const Eigen::VectorXd & q1)
  {
    Eigen::VectorXd distances(model.nbody-1);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      DistanceStep::run(model.joints[i],
                        DistanceStep::ArgsType (i-1, q0, q1, distances)
                        );
    }
    return distances;
  }

  struct RandomStep : public fusion::JointModelVisitor<RandomStep>
  {
    typedef boost::fusion::vector<Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(RandomStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     Eigen::VectorXd & q) 
    {
      jmodel.jointConfigSelector(q) = jmodel.random();
    }

  };

  inline Eigen::VectorXd
  random(const Model & model)
  {
    Eigen::VectorXd q(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      RandomStep::run(model.joints[i],
                       RandomStep::ArgsType (q)
                       );
    }
    return q;
  }

  struct UniformlySample : public fusion::JointModelVisitor<UniformlySample>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(UniformlySample);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     const se3::Model & model,
                     Eigen::VectorXd & q) 
    {
      jmodel.jointConfigSelector(q) = jmodel.uniformlySample(jmodel.jointConfigSelector(model.lowerPositionLimit),
                                                              jmodel.jointConfigSelector(model.upperPositionLimit)
                                                              );
    }

  };

  inline Eigen::VectorXd
  uniformlySample(const Model & model)
  {
    Eigen::VectorXd q(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.nbody; ++i )
    {
      UniformlySample::run(model.joints[i],
                           UniformlySample::ArgsType (model, q)
                           );
    }
    return q;
  }
} // namespace se3

#endif // ifndef __se3_joint_configuration_hpp__

