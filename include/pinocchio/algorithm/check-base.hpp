//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_algorithm_check_base_hpp__
#define __pinocchio_algorithm_check_base_hpp__

#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{

  /// CRTP class describing the API of the checkers
  template<typename AlgorithmCheckerDerived>
  struct AlgorithmCheckerBase
  {
    AlgorithmCheckerDerived & derived()
    {
      return *static_cast<AlgorithmCheckerDerived *>(this);
    }

    const AlgorithmCheckerDerived & derived() const
    {
      return *static_cast<const AlgorithmCheckerDerived *>(this);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    bool checkModel(const ModelTpl<Scalar, Options, JointCollectionTpl> & model) const
    {
      return derived().checkModel_impl(model);
    }
  };

} // namespace pinocchio

#endif // __pinocchio_algorithm_check_base_hpp__
