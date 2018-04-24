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

#ifndef __se3_lie_group_variant_visitor_hpp__
#define __se3_lie_group_variant_visitor_hpp__

#include "pinocchio/multibody/liegroup/liegroup-variant.hpp"

namespace se3
{
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group configuration space
   *
   * @param[in]  lg  The LieGroupVariant.
   *
   * @return     The dimension of the Lie group configuration space
   */
  inline int nq(const LieGroupVariant & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group tangent space
   *
   * @param[in]  lg  The LieGroupVariant.
   *
   * @return     The dimension of the Lie group tangent space
   */
  inline int nv(const LieGroupVariant & lg);
  
}

/// Details
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hxx"

#endif // ifndef __se3_lie_group_variant_visitor_hpp__
