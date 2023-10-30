//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_default_check_hpp__
#define __pinocchio_default_check_hpp__

#include <pinocchio/algorithm/check.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>

namespace pinocchio
{
  /// Default checker-list, used as the default argument in Model::check().
  inline AlgorithmCheckerList<ParentChecker,CRBAChecker,ABAChecker> makeDefaultCheckerList()
  { return makeAlgoCheckerList(ParentChecker(),CRBAChecker(),ABAChecker()); }

#define DEFAULT_CHECKERS makeDefaultCheckerList()

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar,Options,JointCollectionTpl>::check() const
  { return this->check(DEFAULT_CHECKERS); }

} // namespace pinocchio 

#endif // ifndef __pinocchio_default_check_hpp__
