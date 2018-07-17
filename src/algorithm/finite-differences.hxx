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

#ifndef __se3_finite_differences_hxx__
#define __se3_finite_differences_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include <boost/foreach.hpp>

/// @cond DEV

namespace se3
{
  namespace details
  {
    struct FinitDiffEpsVisitor : public fusion::JointVisitorBase<FinitDiffEpsVisitor>
    {
      typedef boost::fusion::vector<
      Eigen::VectorXd &
      > ArgsType;
      
      template<typename JointModel>
      static void algo(const se3::JointModelBase<JointModel> & jmodel,
                       Eigen::VectorXd & fd_increment)
      {
        jmodel.jointVelocitySelector(fd_increment).fill(jmodel.finiteDifferenceIncrement());
      }
      
    }; // struct FinitDiffEpsVisitor
    
  } // namespace details
  
  inline Eigen::VectorXd finiteDifferenceIncrement(const Model & model)
  {
    using namespace se3::details;
    Eigen::VectorXd fd_increment(model.nv);
    for(std::size_t k = 1; k < model.joints.size(); ++k)
    {
      const JointModel & jmodel = model.joints[k];
      FinitDiffEpsVisitor::run(jmodel,FinitDiffEpsVisitor::ArgsType(fd_increment));
    }
    
    return fd_increment;
  }
} // namespace se3

/// @endcond

#endif // ifndef __se3_finite_differences_hxx__
