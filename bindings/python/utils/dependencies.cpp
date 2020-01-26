//
// Copyright (c) 2020 INRIA
//

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;

    void exposeDependencies()
    {
      bp::scope().attr("WITH_HPP_FCL") =
#ifdef PINOCCHIO_WITH_HPP_FCL
      true;
#else
      false;
#endif
      
      bp::scope().attr("WITH_URDFDOM") =
#ifdef PINOCCHIO_WITH_URDFDOM
      true;
#else
      false;
#endif
      
      bp::scope().attr("WITH_CPPAD") =
#ifdef PINOCCHIO_WITH_CPPAD
      true;
#else
      false;
#endif
    }
    
  } // namespace python
} // namespace pinocchio
