//
// Copyright (c) 2023-2024 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  // --------------------------------------------------------------
  // Traits
  // --------------------------------------------------------------
  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct traits<ConstraintDataTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  : traits<ConstraintModelTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  {
  };

  // --------------------------------------------------------------
  // Struct
  // --------------------------------------------------------------
  template<
    typename _Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintDataTpl
  : ConstraintDataBase<ConstraintDataTpl<_Scalar, _Options, ConstraintCollectionTpl>>
  , ConstraintCollectionTpl<_Scalar, _Options>::ConstraintDataVariant
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef ConstraintDataTpl Self;
    typedef ConstraintDataBase<Self> Base;
    typedef ConstraintDataBase<Self> RootBase;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Self>::ConstraintModel ConstraintModel;
    typedef typename traits<Self>::ConstraintData ConstraintData;

    typedef typename traits<Self>::Scalar Scalar;
    static constexpr int Options = traits<Self>::Options;

    // Variant related ----------------------------------------------
    typedef ConstraintCollectionTpl<Scalar, Options> ConstraintCollection;
    typedef typename ConstraintCollection::ConstraintModelVariant ConstraintModelVariant;
    typedef typename ConstraintCollection::ConstraintDataVariant ConstraintDataVariant;

    // -------------------------------
    // METHODS SPECIFIC TO CLASS
    // -------------------------------

    // CRTP related ------------------

    /// \brief Cast to variant.
    ConstraintDataVariant & toVariant()
    {
      return static_cast<ConstraintDataVariant &>(*this);
    }

    /// \brief Const cast to variant.
    const ConstraintDataVariant & toVariant() const
    {
      return static_cast<const ConstraintDataVariant &>(*this);
    }

    // Constructors ------------------

    /// \brief Default constructor
    ConstraintDataTpl()
    : ConstraintDataVariant()
    {
    }

    /// \brief Constructor from a variant.
    ConstraintDataTpl(const ConstraintDataVariant & cdata_variant)
    : ConstraintDataVariant(cdata_variant)
    {
    }

    /// \brief Constructor from a constraint data.
    template<typename ContraintDataDerived>
    ConstraintDataTpl(const ConstraintDataBase<ContraintDataDerived> & cdata)
    : ConstraintDataVariant((ConstraintDataVariant)cdata.derived())
    {
      BOOST_MPL_ASSERT(
        (boost::mpl::contains<typename ConstraintDataVariant::types, ContraintDataDerived>));
    }

    // Operators ---------------------

    /// \brief Is this equal to other?
    template<typename ConstraintDataDerived>
    bool isEqual(const ConstraintDataBase<ConstraintDataDerived> & other) const
    {
      return ::pinocchio::isEqual(*this, other.derived());
    }

    /// \brief Is this equal to other?
    bool isEqual(const ConstraintDataTpl & other) const
    {
      return /*Base::isEqual(other) &&*/ toVariant() == other.toVariant();
    }

    /// \brief Comparison operator
    bool operator==(const ConstraintDataTpl & other) const
    {
      return isEqual(other);
    }

    /// \brief Comparison operator
    bool operator!=(const ConstraintDataTpl & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // IMPLEMENTATIONS OF BASE METHODS
    // -------------------------------

    /// \copydoc Base::classname
    static std::string classnameImpl()
    {
      return "ConstraintData";
    }

    /// \copydoc Base::shortname
    std::string shortnameImpl() const
    {
      return ::pinocchio::visitors::shortname(*this);
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
