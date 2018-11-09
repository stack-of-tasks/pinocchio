//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_lua_hpp__
#define __pinocchio_lua_hpp__

#include <string>
#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{
  namespace lua
  {
    Model buildModel (const std::string & filename, bool freeFlyer = false, bool verbose = false);
  } // namespace lua
} // namespace pinocchio

#endif // ifndef __pinocchio_lua_hpp__
