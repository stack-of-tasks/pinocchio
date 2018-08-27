//
// Copyright (c) 2016-2018 CNRS
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

#include <Eigen/Core>
#include "pinocchio/multibody/fwd.hpp"

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
  template<typename LieGroup_t>
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
  template<typename LieGroup_t>
  inline Eigen::VectorXd interpolate(const Model & model,
                                     const Eigen::VectorXd & q0,
                                     const Eigen::VectorXd & q1,
                                     const double u);

  /**
   * @brief      Compute the tangent vector that must be integrated during one unit time to go from q0 to q1
   *
   * @param[in]  model   Model to be differenced
   * @param[in]  q0      Initial configuration (size model.nq)
   * @param[in]  q1      Wished configuration (size model.nq)
   * @return     The corresponding velocity (size model.nv)
   */
  template<typename LieGroup_t>
  inline Eigen::VectorXd difference(const Model & model,
                                    const Eigen::VectorXd & q0,
                                    const Eigen::VectorXd & q1);


  /**
   * @brief      Squared distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @return     The corresponding squared distances for each joint (size model.njoints-1 = number of joints)
   */
  template<typename LieGroup_t>
  inline Eigen::VectorXd squaredDistance(const Model & model,
                                         const Eigen::VectorXd & q0,
                                         const Eigen::VectorXd & q1);
  /**
   * @brief      Distance between two configuration vectors
   *
   * @param[in]  model      Model we want to compute the distance
   * @param[in]  q0         Configuration 0 (size model.nq)
   * @param[in]  q1         Configuration 1 (size model.nq)
   * @return     The distance between the two configurations
   */
  template<typename LieGroup_t>
  inline double
  distance(const Model & model,
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
  template<typename LieGroup_t>
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
  template<typename LieGroup_t>
  inline Eigen::VectorXd randomConfiguration(const Model & model);

  /**
   * @brief         Normalize a configuration
   *
   * @param[in]     model      Model
   * @param[in,out] q          Configuration to normalize
   */
  template<typename LieGroup_t>
  inline void normalize(const Model & model,
                        Eigen::VectorXd & q);
  
  /**
   * @brief         Return true if the given configurations are equivalents
   * \warning       Two configurations can be equivalent but not equally coefficient wise (e.g for quaternions)
   *
   * @param[in]     model     Model
   * @param[in]     q1        The first configuraiton to compare
   * @param[in]     q2        The Second configuraiton to compare
   * @param[in]     prec      precision of the comparison
   *
   * @return     Wheter the configurations are equivalent or not
   */
  template<typename LieGroup_t>
  inline bool isSameConfiguration(const Model & model,
                                  const Eigen::VectorXd & q1,
                                  const Eigen::VectorXd & q2,
                                  const double & prec = Eigen::NumTraits<double>::dummy_precision());
  
  /**
   * @brief         Return the neutral configuration element related to the model configuration space.
   *
   * @param[in]     model      Model
   *
   * @return        The neutral configuration element.
   */
  template<typename LieGroup_t>
  inline Eigen::VectorXd neutral(const Model & model);
  
  
  
} // namespace se3

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/joint-configuration.hxx"

#endif // ifndef __se3_joint_configuration_hpp__

