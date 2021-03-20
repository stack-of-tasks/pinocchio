//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_serialization_fwd_hpp__
#define __pinocchio_serialization_fwd_hpp__

#include "pinocchio/fwd.hpp"
#include <boost/serialization/nvp.hpp>

#include "pinocchio/serialization/eigen.hpp"

#define BOOST_SERIALIZATION_MAKE_NVP(member) boost::serialization::make_nvp(##member,member)

namespace pinocchio
{
  template<typename T>
  struct Serialize
  {
    template<typename Archive>
    static void run(Archive & ar, T & object);
  };
}

#endif // ifndef __pinocchio_serialization_fwd_hpp__
