//
// Copyright (c) 2016-2018 CNRS
//

#ifndef __pinocchio_check_hpp__
#define __pinocchio_check_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include <boost/fusion/container/list.hpp>
#include <boost/fusion/container/generation/make_list.hpp>

#ifndef PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE
#define PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE 5
#endif

namespace pinocchio
{

  /// CRTP class describing the API of the checkers 
  template<typename AlgorithmCheckerDerived>
  struct AlgorithmCheckerBase
  {
    inline AlgorithmCheckerDerived&       derived()       
    { return *static_cast<      AlgorithmCheckerDerived*>(this); }

    inline const AlgorithmCheckerDerived& derived() const 
    { return *static_cast<const AlgorithmCheckerDerived*>(this); }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    inline bool checkModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const
    { return derived().checkModel_impl(model); }
  };

#define PINOCCHIO_DEFINE_ALGO_CHECKER(NAME)                                           \
  struct NAME##Checker : public AlgorithmCheckerBase<NAME##Checker>         \
  {                                                                         \
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>                                      \
    inline bool checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> &) const;   \
  }

  /// Simple model checker, that assert that model.parents is indeed a tree.
  PINOCCHIO_DEFINE_ALGO_CHECKER(Parent);
  
#if !defined(BOOST_FUSION_HAS_VARIADIC_LIST)
  /// Checker having a list of Checker as input argument
  template<BOOST_PP_ENUM_PARAMS_WITH_A_DEFAULT(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE,class D,boost::fusion::void_)>
  struct AlgorithmCheckerList: AlgorithmCheckerBase< AlgorithmCheckerList<BOOST_PP_ENUM_PARAMS(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE,D)> >
  {
    typedef typename boost::fusion::list<BOOST_PP_ENUM_PARAMS(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE,D)> ArgType;
    
    AlgorithmCheckerList(const ArgType & checkerList)
    : checkerList(checkerList) {}
   
    // Calls model.check for each checker in the fusion::list.
    // Each list element is supposed to implement the AlgorithmCheckerBase API.
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bool checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const;
      
    const ArgType & checkerList;
  };

#define MAKE_ALGO_CHECKER_LIST(z,n,_) \
  /**/ \
  template<BOOST_PP_ENUM_PARAMS(BOOST_PP_INC(n),class D)> \
  AlgorithmCheckerList<BOOST_PP_ENUM_PARAMS(BOOST_PP_INC(n),D)> makeAlgoCheckerList(BOOST_PP_ENUM_BINARY_PARAMS(BOOST_PP_INC(n) , D, const & arg)) \
  { return AlgorithmCheckerList<BOOST_PP_ENUM_PARAMS(BOOST_PP_INC(n),D)>(boost::fusion::make_list(BOOST_PP_ENUM_PARAMS(BOOST_PP_INC(n),arg))); } \

  BOOST_PP_REPEAT(PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE, MAKE_ALGO_CHECKER_LIST, BOOST_PP_EMPTY)
#else
  template <class ...D>
  struct AlgorithmCheckerList: AlgorithmCheckerBase< AlgorithmCheckerList<D...> >
  {
    typedef typename boost::fusion::list<D...> ArgType;
    
    AlgorithmCheckerList(const ArgType & checkerList)
    : checkerList(checkerList) {}
    
    // Calls model.check for each checker in the fusion::list.
    // Each list element is supposed to implement the AlgorithmCheckerBase API.
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bool checkModel_impl(const ModelTpl<Scalar,Options,JointCollectionTpl> & model) const;
    
    const ArgType & checkerList;
  };
  
  template <class ...T>
  AlgorithmCheckerList<T...> makeAlgoCheckerList(const T&... args)
  {
    return AlgorithmCheckerList<T...>(boost::fusion::make_list(args...));
  }
  
#endif

  /// Check the validity of data wrt to model, in particular if model has been modified.
  ///
  /// \param[in] model reference model
  /// \param[in] data corresponding data
  ///
  /// \returns True if data is valid wrt model.
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline bool checkData(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data);

} // namespace pinocchio 


  /* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/check.hxx"

#endif // ifndef __pinocchio_check_hpp__
