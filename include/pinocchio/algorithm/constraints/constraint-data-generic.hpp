//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_algorithm_constraint_data_generic_hpp__
#define __pinocchio_algorithm_constraint_data_generic_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/algorithm/constraints/constraint-model-base.hpp"
#include "pinocchio/algorithm/constraints/constraint-data-base.hpp"
#include "pinocchio/algorithm/constraints/visitors/constraint-model-visitor.hpp"

namespace pinocchio
{

  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct traits<ConstraintDataTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintModel;
  };

  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintDataTpl
  : ConstraintDataBase<ConstraintDataTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  , ConstraintCollectionTpl<_Scalar, _Options>::ConstraintDataVariant
  {
    typedef ConstraintDataBase<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>> Base;
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef ConstraintCollectionTpl<Scalar, Options> ConstraintCollection;
    typedef typename ConstraintCollection::ConstraintDataVariant ConstraintDataVariant;
    typedef typename ConstraintCollection::ConstraintModelVariant ConstraintModelVariant;

    ConstraintDataTpl()
    : ConstraintDataVariant()
    {
    }

    ConstraintDataTpl(const ConstraintDataVariant & cdata_variant)
    : ConstraintDataVariant(cdata_variant)
    {
    }

    template<typename ContraintDataDerived>
    ConstraintDataTpl(const ConstraintDataBase<ContraintDataDerived> & cdata)
    : ConstraintDataVariant((ConstraintDataVariant)cdata.derived())
    {
      BOOST_MPL_ASSERT(
        (boost::mpl::contains<typename ConstraintDataVariant::types, ContraintDataDerived>));
    }

    ConstraintDataVariant & toVariant()
    {
      return static_cast<ConstraintDataVariant &>(*this);
    }

    const ConstraintDataVariant & toVariant() const
    {
      return static_cast<const ConstraintDataVariant &>(*this);
    }

    template<typename ConstraintDataDerived>
    bool isEqual(const ConstraintDataBase<ConstraintDataDerived> & other) const
    {
      return ::pinocchio::isEqual(*this, other.derived());
    }

    bool isEqual(const ConstraintDataTpl & other) const
    {
      return /*Base::isEqual(other) &&*/ toVariant() == other.toVariant();
    }

    bool operator==(const ConstraintDataTpl & other) const
    {
      return isEqual(other);
    }

    bool operator!=(const ConstraintDataTpl & other) const
    {
      return !(*this == other);
    }
  };

  template<
    typename ConstraintDataDerived,
    typename Scalar,
    int Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  bool operator==(
    const ConstraintDataBase<ConstraintDataDerived> & data1,
    const ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & data2)
  {
    return data2 == data1.derived();
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_data_generic_hpp__
