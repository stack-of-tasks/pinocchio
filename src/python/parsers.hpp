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

#include "pinocchio/multibody/parser/urdf.hpp"

namespace se3
{
  namespace python
  {
    struct ParsersPythonVisitor
    {
      static ModelHandler buildModelFromUrdf( const std::string & filename,
					      bool ff )
      {
	Model * model = new Model();
	*model = se3::urdf::buildModel(filename,ff);
	return ModelHandler(model,true);
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::def("buildModelFromUrdf",buildModelFromUrdf,
		bp::args("Filename (string)",
			 "Free flyer (bool, false for a fixed robot)"),
		"Parse the urdf file given in input and return a proper pinocchio model "
		"(remember to create the corresponding data structure).");
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

