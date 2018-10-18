//
// Copyright (c) 2018 CNRS, INRIA
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

#include "pinocchio/bindings/python/utils/constant.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"
#include "pinocchio/utils/version.hpp"

#include <boost/python.hpp>

namespace se3
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    BOOST_PYTHON_FUNCTION_OVERLOADS(printVersion_overload, printVersion, 0, 1);
    
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
} // namespace se3
