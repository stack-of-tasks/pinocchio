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

#include "pinocchio/multibody/liegroup/fwd.hpp"

namespace se3
{
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group configuration space
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The dimension of the Lie group configuration space
   */
  template<typename LieGroupCollection>
  inline int nq(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the dimension of
   *             the Lie group tangent space
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The dimension of the Lie group tangent space
   */
  template<typename LieGroupCollection>
  inline int nv(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the name of it
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The Lie group name
   */
  template<typename LieGroupCollection>
  inline std::string name(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to get the neutral element of it
   *
   * @param[in]  lg  the LieGroupVariant.
   *
   * @return     The Lie group neutral element
   */
  template<typename LieGroupCollection>
  inline Eigen::Matrix<typename LieGroupCollection::Scalar,Eigen::Dynamic,1,LieGroupCollection::Options>
  neutral(const LieGroupGenericTpl<LieGroupCollection> & lg);
  
  /**
   * @brief      Visit a LieGroupVariant to call its integrate method
   *
   * @param[in]  lg  the LieGroupVariant.
   * @param[in]  q   the starting configuration.
   * @param[in]  v   the tangent velocity.
   *
   */
  template<typename LieGroupCollection, class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  inline void integrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
                        const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Tangent_t>  & v,
                        const Eigen::MatrixBase<ConfigOut_t>& qout);
  
}

/// Details
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hxx"

#endif // ifndef __se3_lie_group_variant_visitor_hpp__
