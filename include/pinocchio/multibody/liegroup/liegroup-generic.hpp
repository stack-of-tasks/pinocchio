//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_lie_group_generic_hpp__
#define __pinocchio_lie_group_generic_hpp__

#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hpp"

namespace pinocchio
{
  template<typename LieGroupCollection> struct LieGroupGenericTpl;
  
  template<typename LieGroupCollection>
  struct traits< LieGroupGenericTpl<LieGroupCollection> >
  {
    typedef typename LieGroupCollection::Scalar Scalar;
    enum {
      Options = LieGroupCollection::Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
  };
  
  template<typename LieGroupCollection>
  struct LieGroupGenericTpl
  : LieGroupBase< LieGroupGenericTpl<LieGroupCollection> >, LieGroupCollection::LieGroupVariant
  {
    typedef typename LieGroupCollection::LieGroupVariant Base;
    typedef typename LieGroupCollection::LieGroupVariant LieGroupVariant;
    
    typedef typename LieGroupCollection::Scalar Scalar;
    enum { Options = LieGroupCollection::Options };
    
    template<typename LieGroupDerived>
    LieGroupGenericTpl(const LieGroupBase<LieGroupDerived> & lg_base)
    : Base(lg_base.derived())
    {}
    
    template<typename LieGroup>
    LieGroupGenericTpl(const LieGroupVariant & lg_variant)
    : Base(lg_variant)
    {}
    
    LieGroupGenericTpl(const LieGroupGenericTpl & lg_generic)
    : Base(lg_generic)
    {}

    LieGroupGenericTpl & operator=(const LieGroupGenericTpl & other)
    {
      static_cast<Base&>(*this) = other.toVariant();
      return *this;
    }

    const LieGroupVariant & toVariant() const
    { return static_cast<const LieGroupVariant &>(*this); }
    
    LieGroupVariant & toVariant()
    { return static_cast<LieGroupVariant &>(*this); }

    bool isEqual_impl (const LieGroupGenericTpl& other) const
    {
      return boost::apply_visitor(visitor::LieGroupEqual<Scalar, Options>(), toVariant(), other.toVariant());
    }

    int nq() const { return ::pinocchio::nq(*this); }
    int nv() const { return ::pinocchio::nv(*this); }

    bool operator== (const LieGroupGenericTpl& other) const
    {
      return isEqual_impl(other);
    }

    bool operator!= (const LieGroupGenericTpl& other) const
    {
      return this->isDifferent_impl(other);
    }
    
    std::string name() const
    {
      return LieGroupNameVisitor::run(*this);
    }
  };
  
}

#endif // ifndef __pinocchio_lie_group_generic_hpp__

