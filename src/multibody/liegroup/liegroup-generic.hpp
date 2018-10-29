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

#ifndef __se3_lie_group_generic_hpp__
#define __se3_lie_group_generic_hpp__

#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hpp"

namespace se3
{
  template<typename LieGroupCollection> struct LieGroupGenericTpl;
  
  template<typename LieGroupCollection>
  struct traits< LieGroupGenericTpl<LieGroupCollection> >
  {
    typedef typename LieGroupCollection::Scalar Scalar;
    enum {
      Options = LieGroupCollection::Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
  };
  
  template<typename LieGroupCollection>
  struct LieGroupGenericTpl
  : LieGroupBase< LieGroupGenericTpl<LieGroupCollection> >, LieGroupCollection::LieGroupVariant
  {
    typedef typename LieGroupCollection::LieGroupVariant Base;
    typedef typename LieGroupCollection::LieGroupVariant LieGroupVariant;
    
    typedef typename LieGroupCollection::Scalar Scalar;
    enum { Options = LieGroupCollection::Options };
    
    template<typename LieGroupDerived>
    LieGroupGenericTpl(const LieGroupBase<LieGroupDerived> & lg_base)
    : Base(lg_base.derived())
    {}
    
    template<typename LieGroup>
    LieGroupGenericTpl(const LieGroupVariant & lg_variant)
    : Base(lg_variant)
    {}
    
    const LieGroupVariant & toVariant() const
    { return static_cast<const LieGroupVariant &>(*this); }
    
    LieGroupVariant & toVariant()
    { return static_cast<LieGroupVariant &>(*this); }
    
    int nq() const { return ::se3::nq(*this); }
    int nv() const { return ::se3::nv(*this); }
  };
  
}

#endif // ifndef __se3_lie_group_generic_hpp__

