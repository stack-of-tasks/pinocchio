//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_check_hpp__
#define __se3_check_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include <boost/fusion/container/list.hpp>
#include <boost/fusion/container/generation/make_list.hpp>

#ifndef PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE
#define PINOCCHIO_ALGO_CHECKER_LIST_MAX_LIST_SIZE 5
#endif

namespace se3
{

  /// CRTP class describing the API of the checkers 
  template<typename AlgorithmCheckerDerived>
  struct AlgorithmCheckerBase
  {
    inline AlgorithmCheckerDerived&       derived()       
    { return *static_cast<      AlgorithmCheckerDerived*>(this); }

    inline const AlgorithmCheckerDerived& derived() const 
    { return *static_cast<const AlgorithmCheckerDerived*>(this); }

    inline bool checkModel(const Model & model) const { return derived().checkModel_impl(model); }
  };

#define DEFINE_ALGO_CHECKER(NAME)                                       \
  struct NAME##Checker : public AlgorithmCheckerBase<NAME##Checker>     \
  {                                                                     \
    inline bool checkModel_impl( const Model& ) const;                  \
  }

  /// Simple model checker, that assert that model.parents is indeed a tree.
  DEFINE_ALGO_CHECKER(Parent);
  
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
    bool checkModel_impl(const Model & model) const;
      
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
    bool checkModel_impl(const Model & model) const;
    
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
  inline bool checkData(const Model & model, const Data & data);

} // namespace se3 


  /* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/check.hxx"

#endif // ifndef __se3_check_hpp__
