#ifndef __se3_lua_hpp__
#define __se3_lua_hpp__

#include <iostream>
#include "pinocchio/multibody/model.hpp"

namespace se3
{
  namespace lua
  {
    Model buildModel (const std::string & filename, bool freeFlyer = false, bool verbose = false);
  } // namespace lua
} // namespace se3

#endif // ifndef __se3_lua_hpp__
