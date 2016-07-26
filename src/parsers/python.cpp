//
// Copyright (c) 2016 CNRS
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

#include "pinocchio/parsers/python.hpp"

#include <iostream>
#include <Python.h>
#include <boost/shared_ptr.hpp>

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    Model buildModel(const std::string & filename, const std::string & model_name, bool verbose) throw (bp::error_already_set)
    {
      Py_Initialize();
      // Get a dict for the global namespace to exec further python code with
      bp::object main_module = bp::import("__main__");
      bp::dict globals = bp::extract<bp::dict>(main_module.attr("__dict__"));

      // We need to link to the pinocchio PyWrap. We delegate the dynamic loading to the python interpreter.
      bp::exec("import pinocchio", globals);

      // Create a new Model, get a shared_ptr to it, and include this pointer
      // into the global namespace. See python/handler.hpp for more details.
      boost::shared_ptr<Model> model(new Model());
      bp::object obj(model);
      globals[model_name] = obj;

      // That's it, you can exec your python script, starting with a model you
      // can update as you want.
      try {
        bp::exec_file((bp::str)filename, globals);
      }
      catch (bp::error_already_set & e)
      {
        PyErr_PrintEx(0);
      }
      
      if (verbose)
      {
        std::cout << "Your model has been built. It has " << model->nv;
        std::cout << " degrees of freedom." << std::endl;
      }
      
      return *model;
    }
  } // namespace python
} // namespace se3
