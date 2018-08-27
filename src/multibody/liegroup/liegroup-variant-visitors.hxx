//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_lie_group_variant_visitor_hxx__
#define __se3_lie_group_variant_visitor_hxx__

#include "pinocchio/multibody/liegroup/operation-base.hpp"
#include "pinocchio/multibody/visitor.hpp"

#include <string>

#define LIE_GROUP_VISITOR(VISITOR) \
VISITOR(ArgsType & args) : args(args) {} \
ArgsType & args

namespace se3
{
  
  namespace visitor
  {
    namespace bf = boost::fusion;
    
    template<typename Visitor>
    struct LieGroupVisitorBase : public boost::static_visitor<>
    {
      template<typename D>
      void operator() (const LieGroupBase<D> & lg) const
      {
        bf::invoke(&Visitor::template algo<D>,
                   bf::append(boost::ref(lg),
                              static_cast<const Visitor*>(this)->args));
      }
      
      template<typename ArgsTmp>
      static void run(const LieGroupVariant & lg,
                      ArgsTmp args)
      {
        return boost::apply_visitor(Visitor(args),lg);
      }
    };
  }
  /**
   * @brief Lie Group visitor of the dimension of the configuration space nq
   */
  struct LieGroupNqVisitor: public boost::static_visitor<int>
  {
    template<typename D>
    int operator()(const LieGroupBase<D> & lg) const
    { return lg.nq(); }
    
    static int run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNqVisitor(), lg ); }
  };
  inline int nq(const LieGroupVariant & lg) { return LieGroupNqVisitor::run(lg); }
  
  /**
   * @brief Lie Group visitor of the dimension of the tangent space nv
   */
  struct LieGroupNvVisitor: public boost::static_visitor<int>
  {
    template<typename D>
    int operator()(const LieGroupBase<D> & lg) const
    { return lg.nv(); }
    
    static int run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNvVisitor(), lg ); }
  };
  inline int nv(const LieGroupVariant & lg) { return LieGroupNvVisitor::run(lg); }
  
  /**
   * @brief Visitor of the Lie Group name
   */
  struct LieGroupNameVisitor: public boost::static_visitor<std::string>
  {
    template<typename D>
    std::string operator()(const LieGroupBase<D> & lg) const
    { return lg.name(); }
    
    static std::string run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNameVisitor(), lg ); }
  };
  inline std::string name(const LieGroupVariant & lg) { return LieGroupNameVisitor::run(lg); }
  
  /**
   * @brief Visitor of the Lie Group neutral element
   */
  template<typename Vector>
  struct LieGroupNeutralVisitor: public boost::static_visitor<Vector>
  {
    template<typename D>
    Vector operator()(const LieGroupBase<D> & lg) const
    { return lg.neutral(); }
    
    static Vector run(const LieGroupVariant & lg)
    { return boost::apply_visitor( LieGroupNeutralVisitor(), lg ); }
  };
  
  inline Eigen::VectorXd neutral(const LieGroupVariant & lg)
  { return LieGroupNeutralVisitor<Eigen::VectorXd>::run(lg); }
  
  /**
   * @brief Visitor of the Lie Group integrate method
   */
  template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  struct LieGroupIntegrateVisitor : visitor::LieGroupVisitorBase< LieGroupIntegrateVisitor<ConfigIn_t,Tangent_t,ConfigOut_t> >
  {
    typedef boost::fusion::vector<const ConfigIn_t &,
                                  const Tangent_t &,
                                  ConfigOut_t &> ArgsType;

    LIE_GROUP_VISITOR(LieGroupIntegrateVisitor);

    template<typename LieGroupDerived>
    static void algo(const LieGroupBase<LieGroupDerived> & lg,
                     const Eigen::MatrixBase<ConfigIn_t> & q,
                     const Eigen::MatrixBase<Tangent_t>  & v,
                     const Eigen::MatrixBase<ConfigOut_t>& qout)
    {
      ConfigOut_t & qout_ = const_cast< ConfigOut_t& >(qout.derived());
      lg.integrate(Eigen::Ref<const typename LieGroupDerived::ConfigVector_t>(q),
                   Eigen::Ref<const typename LieGroupDerived::TangentVector_t>(v),
                   Eigen::Ref<typename LieGroupDerived::ConfigVector_t>(qout_));
    }
  };
  
  template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  inline void integrate(const LieGroupVariant & lg,
                        const Eigen::MatrixBase<ConfigIn_t> & q,
                        const Eigen::MatrixBase<Tangent_t>  & v,
                        const Eigen::MatrixBase<ConfigOut_t>& qout)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(ConfigIn_t)
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Tangent_t)
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(ConfigOut_t)
    
    typedef LieGroupIntegrateVisitor<ConfigIn_t,Tangent_t,ConfigOut_t> Operation;
    assert(q.size() == nq(lg));
    assert(v.size() == nv(lg));
    assert(qout.size() == nq(lg));
    
    ConfigOut_t & qout_ = const_cast< ConfigOut_t& >(qout.derived());
    Operation::run(lg,typename Operation::ArgsType(q.derived(),v.derived(),qout_.derived()));
  }
}

#endif // ifndef __se3_lie_group_variant_visitor_hxx__

