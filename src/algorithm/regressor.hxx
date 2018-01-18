//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_regressor_hxx__
#define __se3_regressor_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace se3
{
  
  namespace regressor
  {
    
    inline Data::Matrix3x &
    computeStaticRegressor(const Model & model,
                           Data & data,
                           const Eigen::VectorXd & q)
    {
#ifndef NDEBUG
      assert(model.check(data) && "data is not consistent with model.");
#endif
      assert(q.size() == model.nq);
      
      typedef Data::Matrix3x Matrix3x;
      typedef SizeDepType<4>::ColsReturn<Matrix3x>::Type ColsBlock;
      
      forwardKinematics(model,data,q);
      
      // Computes the total mass of the system
      double mass = 0.;
      for(int i = 1; i < model.njoints; ++i)
        mass += model.inertias[(size_t)i].mass();
      
      const double mass_inv = 1./mass;
      for(int i = 1; i < model.njoints; ++i)
      {
        const SE3 & oMi = data.oMi[(size_t)i];
        ColsBlock sr_cols = data.staticRegressor.middleCols<4>((i-1)*4);
        sr_cols.col(0) = oMi.translation();
        sr_cols.rightCols<3>() = oMi.rotation();
        sr_cols *= mass_inv;
      }
      
      return data.staticRegressor;
    }
  }
  
} // namespace se3

#endif // ifndef __se3_regressor_hxx__
