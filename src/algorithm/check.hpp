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

#ifndef __se3_check_hpp__
#define __se3_check_hpp__

namespace se3
{

  /// CRTP class describing the API of the checkers 
  template<typename AlgorithmCheckerDerived>
  struct AlgorithmCheckerBase
  {
    inline AlgorithmCheckerDerived&       derived()       
    { return *static_cast<      AlgorithmCheckerDerived*>(this); }

    inline const AlgorithmCheckerDerived& derived() const 
    { return *static_cast<const AlgorithmCheckerDerived*>(this); }

    inline bool checkModel(const Model & model) const { return derived().checkModel_impl(model); }
  };

#define DEFINE_ALGO_CHECKER(NAME)                                       \
  struct NAME##Checker : public AlgorithmCheckerBase<NAME##Checker>     \
  {                                                                     \
    inline bool checkModel_impl( const Model& ) const;                  \
  }

  /// Simple model checker, that assert that model.parents is indeed a tree.
  DEFINE_ALGO_CHECKER(Parent);

  /// Check the validity of data wrt to model, in particular if model has been modified.
  ///
  /// \param[in] model reference model
  /// \param[in] data corresponding data
  inline bool checkData(const Model & model, const Data & data);

} // namespace se3 


  /* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/check.hxx"

#endif // ifndef __se3_check_hpp__
