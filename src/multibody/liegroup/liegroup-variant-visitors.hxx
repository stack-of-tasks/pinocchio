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

#ifndef __se3_lie_group_variant_visitor_hxx__
#define __se3_lie_group_variant_visitor_hxx__

#include "pinocchio/multibody/liegroup/operation-base.hpp"

namespace se3
{
  /**
   * @brief Lie Group visitor of the dimension of the configuration space nq
   */
  class LieGroupNqVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const LieGroupOperationBase<D> & lg) const
    { return lg.nq(); }
    
    static int run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNqVisitor(), lg ); }
  };
  inline int nq(const LieGroupVariant & lg) { return LieGroupNqVisitor::run(lg); }
  
  /**
   * @brief Lie Group visitor of the dimension of the tangent space nv
   */
  class LieGroupNvVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const LieGroupOperationBase<D> & lg) const
    { return lg.nv(); }
    
    static int run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNvVisitor(), lg ); }
  };
  inline int nv(const LieGroupVariant & lg) { return LieGroupNvVisitor::run(lg); }
  
}

#endif // ifndef __se3_lie_group_variant_visitor_hxx__

