//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_algorithm_check_model_hxx__
#define __pinocchio_algorithm_check_model_hxx__

#include "pinocchio/multibody/model.hpp"

#include <boost/fusion/algorithm.hpp>

namespace pinocchio
{
  namespace internal
  {
    // Dedicated structure for the fusion::accumulate algorithm: validate the check-algorithm
    // for all elements in a fusion list of AlgoCheckers.
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    struct AlgoFusionChecker
    {
      typedef bool result_type;
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      const Model & model;

      AlgoFusionChecker(const Model & model)
      : model(model)
      {
      }

      bool operator()(const bool & accumul, const boost::fusion::void_ &) const
      {
        return accumul;
      }

      template<typename T>
      bool operator()(const bool & accumul, const AlgorithmCheckerBase<T> & t) const
      {
        return accumul && t.checkModel(model);
      }
    };
  } // namespace internal

  // Check the validity of the kinematic tree defined by parents.
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool
  ParentChecker::checkModel_impl(const ModelTpl<Scalar, Options, JointCollectionTpl> & model) const
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    for (JointIndex j = 1; j < (JointIndex)model.njoints; ++j)
      if (model.parents[j] >= j)
        return false;

    return true;
  }

  // Check if there is a mimic joint in the kinematic tree
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool
  MimicChecker::checkModel_impl(const ModelTpl<Scalar, Options, JointCollectionTpl> & model) const
  {
    if (!model.mimicking_joints.empty())
      return false;

    return true;
  }

  template<class... T>
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool AlgorithmCheckerList<T...>::checkModel_impl(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model) const
  {
    return boost::fusion::accumulate(
      checkerList, true, internal::AlgoFusionChecker<Scalar, Options, JointCollectionTpl>(model));
  }

} // namespace pinocchio

#endif // __pinocchio_algorithm_check_model_hxx__
