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

#ifndef __se3_energy_hpp__
#define __se3_energy_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace se3 {
  inline double
  kineticEnergy(const Model & model,
                Data & data,
                const Eigen::VectorXd & q,
                const Eigen::VectorXd & v,
                const bool update_kinematics = true);
}

/* --- Details -------------------------------------------------------------------- */
namespace se3
{
   
  inline double
  kineticEnergy(const Model & model,
                Data & data,
                const Eigen::VectorXd & q,
                const Eigen::VectorXd & v,
                const bool update_kinematics)
  {
    data.kinetic_energy = 0.;
    
    if (update_kinematics)
      kinematics(model,data,q,v);
    
    for(Model::Index i=1;i<(Model::Index)(model.nbody);++i)
      data.kinetic_energy += model.inertias[i].vtiv(data.v[i]);
    
    data.kinetic_energy *= .5;
    return data.kinetic_energy;
  }
}
#endif // __se3_energy_hpp__
