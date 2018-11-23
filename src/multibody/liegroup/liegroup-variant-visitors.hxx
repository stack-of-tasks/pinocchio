//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_lie_group_variant_visitor_hxx__
#define __pinocchio_lie_group_variant_visitor_hxx__

#include "pinocchio/multibody/liegroup/liegroup-base.hpp"
#include "pinocchio/multibody/visitor.hpp"

#include <string>

#define LIE_GROUP_VISITOR(VISITOR) \
VISITOR(ArgsType & args) : args(args) {} \
ArgsType & args

namespace pinocchio
{
  
  namespace visitor
  {
    namespace bf = boost::fusion;
    
    template<typename Visitor>
    struct LieGroupVisitorBase : public boost::static_visitor<>
    {
      template<typename LieGroupDerived>
      void operator() (const LieGroupBase<LieGroupDerived> & lg) const
      {
        bf::invoke(&Visitor::template algo<LieGroupDerived>,
                   bf::append(boost::ref(lg),
                              static_cast<const Visitor*>(this)->args));
      }
      
      template<typename LieGroupCollection, typename ArgsTmp>
      static void run(const LieGroupGenericTpl<LieGroupCollection> & lg,
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
    template<typename LieGroupDerived>
    int operator()(const LieGroupBase<LieGroupDerived> & lg) const
    { return lg.nq(); }
    
    template<typename LieGroupCollection>
    static int run(const LieGroupGenericTpl<LieGroupCollection> & lg)
    { return boost::apply_visitor( LieGroupNqVisitor(), lg ); }
  };
  
  template<typename LieGroupCollection>
  inline int nq(const LieGroupGenericTpl<LieGroupCollection> & lg)
  { return LieGroupNqVisitor::run(lg); }
  
  /**
   * @brief Lie Group visitor of the dimension of the tangent space nv
   */
  struct LieGroupNvVisitor: public boost::static_visitor<int>
  {
    template<typename LieGroupDerived>
    int operator()(const LieGroupBase<LieGroupDerived> & lg) const
    { return lg.nv(); }
    
    template<typename LieGroupCollection>
    static int run(const LieGroupGenericTpl<LieGroupCollection> & lg)
    { return boost::apply_visitor( LieGroupNvVisitor(), lg ); }
  };
  
  template<typename LieGroupCollection>
  inline int nv(const LieGroupGenericTpl<LieGroupCollection> & lg)
  { return LieGroupNvVisitor::run(lg); }
  
  /**
   * @brief Visitor of the Lie Group name
   */
  struct LieGroupNameVisitor: public boost::static_visitor<std::string>
  {
    template<typename LieGroupDerived>
    std::string operator()(const LieGroupBase<LieGroupDerived> & lg) const
    { return lg.name(); }
    
    template<typename LieGroupCollection>
    static std::string run(const LieGroupGenericTpl<LieGroupCollection> & lg)
    { return boost::apply_visitor( LieGroupNameVisitor(), lg ); }
  };
  
  template<typename LieGroupCollection>
  inline std::string name(const LieGroupGenericTpl<LieGroupCollection> & lg)
  { return LieGroupNameVisitor::run(lg); }
  
  /**
   * @brief Visitor of the Lie Group neutral element
   */
  template<typename Vector>
  struct LieGroupNeutralVisitor: public boost::static_visitor<Vector>
  {
    template<typename LieGroupDerived>
    Vector operator()(const LieGroupBase<LieGroupDerived> & lg) const
    { return lg.neutral(); }
    
    template<typename LieGroupCollection>
    static Vector run(const LieGroupGenericTpl<LieGroupCollection> & lg)
    { return boost::apply_visitor( LieGroupNeutralVisitor(), lg ); }
  };
  
  template<typename LieGroupCollection>
  inline Eigen::Matrix<typename LieGroupCollection::Scalar,Eigen::Dynamic,1,LieGroupCollection::Options>
  neutral(const LieGroupGenericTpl<LieGroupCollection> & lg)
  {
    typedef Eigen::Matrix<typename LieGroupCollection::Scalar,Eigen::Dynamic,1,LieGroupCollection::Options> ReturnType;
    return LieGroupNeutralVisitor<ReturnType>::run(lg);
  }
  
  /**
   * @brief Visitor of the Lie Group integrate method
   */
  template <class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  struct LieGroupIntegrateVisitor
  : visitor::LieGroupVisitorBase< LieGroupIntegrateVisitor<ConfigIn_t,Tangent_t,ConfigOut_t> >
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
  
  template<typename LieGroupCollection, class ConfigIn_t, class Tangent_t, class ConfigOut_t>
  inline void integrate(const LieGroupGenericTpl<LieGroupCollection> & lg,
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

#endif // ifndef __pinocchio_lie_group_variant_visitor_hxx__

