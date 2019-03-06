//
// Copyright (c) 2019 INRIA
//

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    static bool WITH_FCL_SUPPORT()
    {
#ifdef PINOCCHIO_WITH_HPP_FCL
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_URDFDOM_SUPPORT()
    {
#ifdef PINOCCHIO_WITH_URDFDOM
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_LUA5_SUPPORT()
    {
#ifdef PINOCCHIO_WITH_LUA5
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_CPPAD_SUPPORT()
    {
#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT
      return true;
#else
      return false;
#endif
    }

    void exposeDependencies()
    {
      
      bp::def("WITH_FCL_SUPPORT",&WITH_FCL_SUPPORT,
              "Returns True if Pinocchio has been built with the FCL support.");
      
      bp::def("WITH_URDFDOM_SUPPORT",&WITH_URDFDOM_SUPPORT,
              "Returns True if Pinocchio has been built with the URDFDOM support.");
      
      bp::def("WITH_LUA5_SUPPORT",&WITH_LUA5_SUPPORT,
              "Returns True if Pinocchio has been built with the LUA 5 support.");
      
      bp::def("WITH_CPPAD_SUPPORT",&WITH_CPPAD_SUPPORT,
              "Returns True if Pinocchio has been built with the CppAD support.");
    }
    
  } // namespace python
} // namespace pinocchio
