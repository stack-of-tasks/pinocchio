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

#ifndef __se3_default_check_hpp__
#define __se3_default_check_hpp__

#include <pinocchio/algorithm/check.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>

namespace se3
{
  /// Default checker-list, used as the default argument in Model::check().
  inline AlgorithmCheckerList<ParentChecker,CRBAChecker,ABAChecker> makeDefaultCheckerList()
  { return makeAlgoCheckerList(ParentChecker(),CRBAChecker(),ABAChecker()); }

#define DEFAULT_CHECKERS makeDefaultCheckerList()

  inline bool Model::check() const { return this->check(DEFAULT_CHECKERS); }

} // namespace se3 

#endif // ifndef __se3_default_check_hpp__


