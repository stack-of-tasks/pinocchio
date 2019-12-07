//
// Copyright (c) 2019 INRIA
//

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    static bool WITH_FCL()
    {
#ifdef PINOCCHIO_WITH_HPP_FCL
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_URDFDOM()
    {
#ifdef PINOCCHIO_WITH_URDFDOM
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_LUA5()
    {
#ifdef PINOCCHIO_WITH_LUA5
      return true;
#else
      return false;
#endif
    }
    
    static bool WITH_CPPAD()
    {
#ifdef PINOCCHIO_WITH_CPPAD
      return true;
#else
      return false;
#endif
    }

    void exposeDependencies()
    {
      
      bp::def("WITH_FCL",&WITH_FCL,
              "Returns True if Pinocchio has been built with the FCL support.");
      
      bp::def("WITH_URDFDOM",&WITH_URDFDOM,
              "Returns True if Pinocchio has been built with the URDFDOM support.");
      
      bp::def("WITH_LUA5",&WITH_LUA5,
              "Returns True if Pinocchio has been built with the LUA 5 support.");
      
      bp::def("WITH_CPPAD",&WITH_CPPAD,
              "Returns True if Pinocchio has been built with the CppAD support.");
    }
    
  } // namespace python
} // namespace pinocchio
