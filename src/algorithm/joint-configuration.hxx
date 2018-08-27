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

#ifndef __se3_joint_configuration_hxx__
#define __se3_joint_configuration_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <cmath>

/* --- Details -------------------------------------------------------------------- */
namespace se3
{
  inline Eigen::VectorXd
  integrate(const Model & model,
            const Eigen::VectorXd & q,
            const Eigen::VectorXd & v)
  {
    return integrate<LieGroupMap>(model, q, v);
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  integrate(const Model & model,
            const Eigen::VectorXd & q,
            const Eigen::VectorXd & v)
  {
    Eigen::VectorXd integ(model.nq);
    typename IntegrateStep<LieGroup_t>::ArgsType args(q, v, integ);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      IntegrateStep<LieGroup_t>::run (model.joints[i], args);
    }
    return integ;
  }

  inline Eigen::VectorXd
  interpolate(const Model & model,
               const Eigen::VectorXd & q0,
               const Eigen::VectorXd & q1,
               const double u)
  {
    return interpolate<LieGroupMap>(model, q0, q1, u);
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  interpolate(const Model & model,
               const Eigen::VectorXd & q0,
               const Eigen::VectorXd & q1,
               const double u)
  {
    Eigen::VectorXd interp(model.nq);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      InterpolateStep<LieGroup_t>::run
        (model.joints[i], typename InterpolateStep<LieGroup_t>::ArgsType (q0, q1, u, interp));
    }
    return interp;
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  difference(const Model & model,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1)
  {
    Eigen::VectorXd diff(model.nv);
    typename DifferenceStep<LieGroup_t>::ArgsType args(q0, q1, diff);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      DifferenceStep<LieGroup_t>::run(model.joints[i], args);
    }
    return diff;
  }

  inline Eigen::VectorXd
  difference(const Model & model,
                     const Eigen::VectorXd & q0,
                     const Eigen::VectorXd & q1)
  {
    return difference<LieGroupMap>(model, q0, q1);
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  squaredDistance(const Model & model,
                  const Eigen::VectorXd & q0,
                  const Eigen::VectorXd & q1)
  {
    Eigen::VectorXd distances(Eigen::VectorXd::Zero(model.njoints-1));
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      typename SquaredDistanceStep<LieGroup_t>::ArgsType args(i-1, q0, q1, distances);
      SquaredDistanceStep<LieGroup_t>::run(model.joints[i], args);
    }
    return distances;
  }

  inline Eigen::VectorXd
  squaredDistance(const Model & model,
                  const Eigen::VectorXd & q0,
                  const Eigen::VectorXd & q1)
  {
    return squaredDistance<LieGroupMap>(model, q0, q1);
  }
  
  template<typename LieGroup_t>
  inline double
  distance(const Model & model,
           const Eigen::VectorXd & q0,
           const Eigen::VectorXd & q1)
  {
    return std::sqrt(squaredDistance<LieGroup_t>(model, q0, q1).sum());
  }
  
  inline double
  distance(const Model & model,
           const Eigen::VectorXd & q0,
           const Eigen::VectorXd & q1)
  {
    return std::sqrt(squaredDistance<LieGroupMap>(model, q0, q1).sum());
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  randomConfiguration(const Model & model, const Eigen::VectorXd & lowerLimits, const Eigen::VectorXd & upperLimits)
  {
    Eigen::VectorXd q(model.nq);
    typename RandomConfigurationStep<LieGroup_t>::ArgsType args(q, lowerLimits, upperLimits);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      RandomConfigurationStep<LieGroup_t>::run(model.joints[i], args);
    }
    return q;
  }

  inline Eigen::VectorXd
  randomConfiguration(const Model & model, const Eigen::VectorXd & lowerLimits, const Eigen::VectorXd & upperLimits)
  {
    return randomConfiguration<LieGroupMap>(model, lowerLimits, upperLimits);
  }

  template<typename LieGroup_t>
  inline Eigen::VectorXd
  randomConfiguration(const Model & model)
  {
    return randomConfiguration<LieGroup_t>(model, model.lowerPositionLimit, model.upperPositionLimit);
  }

  inline Eigen::VectorXd
  randomConfiguration(const Model & model)
  {
    return randomConfiguration<LieGroupMap>(model);
  }

  template<typename LieGroup_t>
  inline void normalize(const Model & model, Eigen::VectorXd & qout)
  {
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i)
    {
      NormalizeStep<LieGroup_t>::run(model.joints[i],
                                     typename NormalizeStep<LieGroup_t>::ArgsType(qout));
    }
  }
  
  inline void normalize(const Model & model, Eigen::VectorXd & qout)
  {
    return normalize<LieGroupMap>(model,qout);
  }

  template<typename LieGroup_t>
  inline bool
  isSameConfiguration(const Model & model,
                      const Eigen::VectorXd & q1,
                      const Eigen::VectorXd & q2,
                      const double& prec)
  {
    bool result = true;
    typename IsSameConfigurationStep<LieGroup_t>::ArgsType args (result, q1, q2, prec);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      IsSameConfigurationStep<LieGroup_t>::run(model.joints[i], args);
      if( !result )
        return false;
    }
    return true;
  }

  inline bool
  isSameConfiguration(const Model & model,
                      const Eigen::VectorXd & q1,
                      const Eigen::VectorXd & q2,
                      const double& prec = Eigen::NumTraits<double>::dummy_precision())
  {
    return isSameConfiguration<LieGroupMap>(model, q1, q2, prec);
  }
  

  
  template<typename LieGroup_t>
  inline Eigen::VectorXd neutral(const Model & model)
  {
    Eigen::VectorXd neutral_elt(model.nq);
    typename NeutralStep<LieGroup_t>::ArgsType args(neutral_elt);
    for( Model::JointIndex i=1; i<(Model::JointIndex) model.njoints; ++i )
    {
      NeutralStep<LieGroup_t>::run(model.joints[i], args);
      
    }
    return neutral_elt;
  }
  
  inline Eigen::VectorXd neutral(const Model & model)
  {
    return neutral<LieGroupMap>(model);
  }


} // namespace se3

#endif // ifndef __se3_joint_configuration_hxx__

