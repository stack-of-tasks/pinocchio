//
// Copyright (c) 2022-2023 INRIA
//

#ifndef __pinocchio_algorithm_constraints_fwd_hpp__
#define __pinocchio_algorithm_constraints_fwd_hpp__

#include "pinocchio/algorithm/fwd.hpp"
#include <boost/variant.hpp>

namespace pinocchio
{
  template<typename Scalar, int Options = 0>
  struct RigidConstraintModelTpl;
  template<typename Scalar, int Options = 0>
  struct RigidConstraintDataTpl;

  template<typename _Scalar, int _Options>
  struct ConstraintCollectionTpl
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef boost::variant<RigidConstraintModel> ConstraintModelVariant;
    typedef boost::variant<RigidConstraintData> ConstraintDataVariant;
  };

  typedef ConstraintCollectionTpl<context::Scalar, context::Options> ConstraintCollection;

  template<typename Scalar, int _Options, template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintModelTpl;
  typedef ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionTpl>
    ConstraintModel;

  template<typename Scalar, int _Options, template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintDataTpl;
  typedef ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionTpl>
    ConstraintData;

  template<typename Scalar>
  struct CoulombFrictionConeTpl;
  typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;

  template<typename Scalar>
  struct DualCoulombFrictionConeTpl;
  typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraints_fwd_hpp__
