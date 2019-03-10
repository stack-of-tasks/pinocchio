//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_serialization_fwd_hpp__
#define __pinocchio_serialization_fwd_hpp__

#include <boost/serialization/nvp.hpp>

#define BOOST_SERIALIZATION_MAKE_NVP(member) boost::serialization::make_nvp(##member,member)

#endif // ifndef __pinocchio_serialization_fwd_hpp__
