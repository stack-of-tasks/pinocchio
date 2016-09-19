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
   * @brief      Integrate a configuration for the specified model for a tangent vector during one unit time
   *
   * @param[in]  model   Model that must be integrated
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
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @param[in]  model   Model to be differentiated
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
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @return     The corresponding distances for each joint (size model.njoints-1 = number of joints)
   */
  inline Eigen::VectorXd distance(const Model & model,
                                  const Eigen::VectorXd & q0,
                                  const Eigen::VectorXd & q1);


  /**
   * @brief      Generate a configuration vector uniformly sampled among provided limits.
   *
   *\warning     If limits are infinite, exceptions may be thrown in the joint implementation of uniformlySample
   *             
   * @param[in]  model        Model we want to generate a configuration vector of
   * @param[in]  lowerLimits  Joints lower limits
   * @param[in]  upperLimits  Joints upper limits
   *
   * @return     The resulted configuration vector (size model.nq)
   */
  inline Eigen::VectorXd randomConfiguration(const Model & model,
                                             const Eigen::VectorXd & lowerLimits,
                                             const Eigen::VectorXd & upperLimits);

  /**
   * @brief      Generate a configuration vector uniformly sampled among the joint limits of the specified Model.
   *
   *\warning     If limits are infinite (no one specified when adding a body or no modification directly in my_model.{lowerPositionLimit,upperPositionLimit},
   *             exceptions may be thrown in the joint implementation of uniformlySample
   *             
   * @param[in]  model   Model we want to generate a configuration vector of
   * @return     The resulted configuration vector (size model.nq)
   */
  inline Eigen::VectorXd randomConfiguration(const Model & model);

  /**
   * @brief         Normalize a configuration
   *
   * @param[in]     model      Model
   * @param[in,out] q          Configuration to normalize
   */
  inline void normalize(const Model & model,
                        Eigen::VectorXd & q);

  /**
   * @brief         Return true if the given configurations are equivalents
   * \warning       Two configurations can be equivalent but not equally coefficient wise (e.g for quaternions)
   * 
   * @param[in]     model      Model
   * @param[in]     q1        The first configuraiton to compare
   * @param[in]     q2        The Second configuraiton to compare
   * 
   * @return     Wheter the configurations are equivalent or not
   */
  inline bool isSameConfiguration(const Model & model,
                                  const Eigen::VectorXd & q1,
                                  const Eigen::VectorXd & q2);
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
      jmodel.jointConfigSelector(result) = jmodel.integrate(q, v);
    }

  };

  inline Eigen::VectorXd
  integrate(const Model & model,
                 const Eigen::VectorXd & q,
                 const Eigen::VectorXd & v)
  {
    Eigen::VectorXd integ(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
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
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
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
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
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
    Eigen::VectorXd distances(model.njoints-1);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      DistanceStep::run(model.joints[i],
                        DistanceStep::ArgsType (i-1, q0, q1, distances)
                        );
    }
    return distances;
  }


  struct RandomConfiguration : public fusion::JointModelVisitor<RandomConfiguration>
  {
    typedef boost::fusion::vector<Eigen::VectorXd &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_MODEL_VISITOR_INIT(RandomConfiguration);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     Eigen::VectorXd & q,
                     const Eigen::VectorXd & lowerLimits,
                     const Eigen::VectorXd & upperLimits) 
    {
      jmodel.jointConfigSelector(q) = jmodel.randomConfiguration(jmodel.jointConfigSelector(lowerLimits),
                                                                  jmodel.jointConfigSelector(upperLimits)
                                                                  );
    }

  };

  inline Eigen::VectorXd
  randomConfiguration(const Model & model, const Eigen::VectorXd & lowerLimits, const Eigen::VectorXd & upperLimits)
  {
    Eigen::VectorXd q(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      RandomConfiguration::run(model.joints[i],
                               RandomConfiguration::ArgsType ( q, lowerLimits, upperLimits)
                               );
    }
    return q;
  }

  inline Eigen::VectorXd
  randomConfiguration(const Model & model)
  {
    return randomConfiguration(model, model.lowerPositionLimit, model.upperPositionLimit);
  }

  struct NormalizeStep : public fusion::JointModelVisitor<NormalizeStep>
  {
    typedef boost::fusion::vector<Eigen::VectorXd &> ArgsType;

    JOINT_MODEL_VISITOR_INIT(NormalizeStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     Eigen::VectorXd & q)
    {
      jmodel.normalize(q);
    }
  };

  inline void
  normalize(const Model & model,
            Eigen::VectorXd & q)
  {
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      NormalizeStep::run(model.joints[i],
                         NormalizeStep::ArgsType (q));
    }
  }


  struct IsSameConfigurationStep : public fusion::JointModelVisitor<IsSameConfigurationStep>
  {
    typedef boost::fusion::vector<bool &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &> ArgsType;

    JOINT_MODEL_VISITOR_INIT(IsSameConfigurationStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     bool & isSame,
                     const Eigen::VectorXd & q1,
                     const Eigen::VectorXd & q2)
    {
      isSame = jmodel.isSameConfiguration(q1,q2);
    }
  };

  inline bool
  isSameConfiguration(const Model & model,
                      const Eigen::VectorXd & q1,
                      const Eigen::VectorXd & q2)
  {
    bool result = false;
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      IsSameConfigurationStep::run(model.joints[i], IsSameConfigurationStep::ArgsType (result,q1,q2)); 
      if( !result )
        return false;
    }
    return true;
  }


} // namespace se3

#endif // ifndef __se3_joint_configuration_hpp__

