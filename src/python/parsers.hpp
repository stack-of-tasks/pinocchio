//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_parsers_hpp__
#define __se3_python_parsers_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#ifdef WITH_URDFDOM
  #include "pinocchio/multibody/parser/urdf.hpp"
#endif

#ifdef WITH_LUA
  #include "pinocchio/multibody/parser/lua.hpp"
#endif // #ifdef WITH_LUA

namespace se3
{
  namespace python
  {
    struct ParsersPythonVisitor
    {

#ifdef WITH_URDFDOM
      static ModelHandler buildModelFromUrdf( const std::string & filename,
					      bool ff )
      {
	Model * model = new Model();
	*model = se3::urdf::buildModel(filename,ff);
	return ModelHandler(model,true);
      }
#endif

#ifdef WITH_LUA
      static ModelHandler buildModelFromLua(const std::string & filename,
                                            bool ff,
                                            bool verbose
                                            )
      {
        Model * model = new Model ();
        *model = se3::lua::buildModel (filename, ff, verbose);
        return ModelHandler (model,true);
      }
#endif // #ifdef WITH_LUA

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        
#ifdef WITH_URDFDOM
	bp::def("buildModelFromUrdf",buildModelFromUrdf,
		bp::args("Filename (string)",
			 "Free flyer (bool, false for a fixed robot)"),
		"Parse the urdf file given in input and return a proper pinocchio model "
		"(remember to create the corresponding data structure).");
#endif

#ifdef WITH_LUA
        bp::def("buildModelFromLua",buildModelFromLua,
                bp::args("Filename (string)",
                         "Free flyer (bool, false for a fixed robot)",
                         "Verbose option "),
                "Parse the urdf file given in input and return a proper pinocchio model "
                "(remember to create the corresponding data structure).");
#endif // #ifdef WITH_LUA
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_parsers_hpp__

