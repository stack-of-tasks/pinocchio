//
// Copyright (c) 2019-2021 INRIA
//

#ifndef __pinocchio_container_boost_container_limits_hpp__
#define __pinocchio_container_boost_container_limits_hpp__

#include "pinocchio/macros.hpp"

//#ifndef PINOCCHIO_WITH_CXX11_SUPPORT

  #define PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE_DEFAULT 30

  #ifndef PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE
    #define PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE_DEFAULT
  #endif

  #define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS

  #if !defined(BOOST_MPL_LIMIT_LIST_SIZE)
    // Check the inclusion order
    #if defined(BOOST_MPL_LIST_HPP_INCLUDED)
      # error "You should include pinocchio before the Boost headers (e.g. #include <pinocchio/fwd.hpp>)"
    #endif

    #define BOOST_MPL_LIMIT_LIST_SIZE PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE

  #elif BOOST_MPL_LIMIT_LIST_SIZE < PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE
    #if defined(BOOST_MPL_LIST_HPP_INCLUDED)
      # error "You should include pinocchio before the Boost headers (e.g. #include <pinocchio/fwd.hpp>)"
    #else
      # error "BOOST_MPL_LIMIT_LIST_SIZE value is lower than the value of PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE"
    #endif
    
  #endif

//#endif // ifndef PINOCCHIO_WITH_CXX11_SUPPORT

#endif // ifndef __pinocchio_container_boost_container_limits_hpp__
