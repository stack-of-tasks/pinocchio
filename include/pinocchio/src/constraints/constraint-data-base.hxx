//
// Copyright (c) 2023-2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  template<typename Derived>
  struct ConstraintDataBase
  : NumericalBase<Derived>
  , DataEntity<Derived>
  , pinocchio::serialization::Serializable<Derived>


  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef DataEntity<Derived> Base;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Derived>::ConstraintModel ConstraintModel;
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef typename traits<Derived>::Scalar Scalar;
    static constexpr int Options = traits<Derived>::Options;

    // -------------------------------
    // METHODS SPECIFIC TO CLASS
    // -------------------------------

    // CRTP related ------------------

    /// \brief Cast to Derived
    Derived & derived()
    {
      return static_cast<Derived &>(*this);
    }

    /// \brief Const cast to Derived
    const Derived & derived() const
    {
      return static_cast<const Derived &>(*this);
    }

    /// \brief Cast to base.
    ConstraintDataBase & base()
    {
      return *this;
    }

    /// \brief Const cast to base.
    const ConstraintDataBase & base() const
    {
      return *this;
    }

    // Constructors ------------------

  protected:
    /// \brief Default constructor
    ConstraintDataBase()
    {
    }

    // Operators ---------------------

  public:
    /// \brief Comparison operator
    template<typename OtherDerived>
    bool operator==(const ConstraintDataBase<OtherDerived> &) const
    {
      return true;
    }

    /// \brief Comparison operator
    template<typename OtherDerived>
    bool operator!=(const ConstraintDataBase<OtherDerived> & other) const
    {
      return !(*this == other);
    }

    // -------------------------------
    // BASE METHODS
    // -------------------------------

    // General -----------------------

    /// \brief Returns the name of the class.
    static std::string classname()
    {
      return Derived::classnameImpl();
    }

    /// \brief Returns the name of the underlying constraint if this is a variant.
    std::string shortname() const
    {
      return derived().shortnameImpl();
    }

    /// \brief Print the name of the class
    void disp(std::ostream & os) const
    {
      using namespace std;
      os << shortname() << endl;
    }

    /// \brief Print the name of the class
    friend std::ostream &
    operator<<(std::ostream & os, const ConstraintDataBase<Derived> & constraint)
    {
      constraint.disp(os);
      return os;
    }
  }; // struct ConstraintDataBase

} // namespace pinocchio
