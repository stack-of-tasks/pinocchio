//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_algorithm_constraint_model_generic_hpp__
#define __pinocchio_algorithm_constraint_model_generic_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/algorithm/constraints/constraint-model-base.hpp"
#include "pinocchio/algorithm/constraints/constraint-data-generic.hpp"
#include "pinocchio/algorithm/constraints/visitors/constraint-model-visitor.hpp"

namespace pinocchio
{

  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct traits<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintData;
  };

  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintModelTpl
  : ConstraintModelBase<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  , ConstraintCollectionTpl<_Scalar, _Options>::ConstraintModelVariant
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef ConstraintModelBase<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
      Base;
    typedef ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintData;
    typedef ConstraintCollectionTpl<Scalar, Options> ConstraintCollection;
    typedef typename ConstraintCollection::ConstraintDataVariant ConstraintDataVariant;
    typedef typename ConstraintCollection::ConstraintModelVariant ConstraintModelVariant;

    ConstraintModelTpl()
    : ConstraintModelVariant()
    {
    }

    ConstraintModelTpl(const ConstraintModelVariant & cmodel_variant)
    : ConstraintModelVariant(cmodel_variant)
    {
    }

    template<typename ContraintModelDerived>
    ConstraintModelTpl(const ConstraintModelBase<ContraintModelDerived> & cmodel)
    : ConstraintModelVariant((ConstraintModelVariant)cmodel.derived())
    {
      BOOST_MPL_ASSERT(
        (boost::mpl::contains<typename ConstraintModelVariant::types, ContraintModelDerived>));
    }

    ConstraintData createData() const
    {
      return ::pinocchio::createData<Scalar, Options, ConstraintCollectionTpl>(*this);
    }
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_model_generic_hpp__
