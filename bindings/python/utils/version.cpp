//
// Copyright (c) 2018 CNRS, INRIA
//

#include "pinocchio/bindings/python/utils/constant.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"
#include "pinocchio/utils/version.hpp"

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(printVersion_overload, printVersion, 0, 1)
    
    void exposeVersion()
    {
      // Define release numbers of the current Pinocchio version.
      bp::def_constant("PINOCCHIO_MAJOR_VERSION",PINOCCHIO_MAJOR_VERSION);
      bp::def_constant("PINOCCHIO_MINOR_VERSION",PINOCCHIO_MINOR_VERSION);
      bp::def_constant("PINOCCHIO_PATCH_VERSION",PINOCCHIO_PATCH_VERSION);
      
      bp::def("printVersion",printVersion,
              printVersion_overload(bp::arg("delimiter"),
                                    "Returns the current version of Pinocchio as a string.\n"
                                    "The user may specify the delimiter between the different semantic numbers.")
              );
      
      bp::def("checkVersionAtLeast",&checkVersionAtLeast,
              bp::args("major","minor","patch"),
              "Checks if the current version of Pinocchio is at least"
              " the version provided by the input arguments.");
    }
    
  } // namespace python
} // namespace pinocchio
