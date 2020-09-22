//
// Copyright (c) 2016-2020 CNRS INRIA
//

#include "pinocchio/parsers/python.hpp"

#include <iostream>
#include <Python.h>
#include <boost/shared_ptr.hpp>
#include <boost/version.hpp>
#include <boost/algorithm/string/predicate.hpp>

// Boost 1.58
#if BOOST_VERSION / 100 % 1000 == 58
#include <fstream>
#endif

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    Model buildModel(const std::string & filename, const std::string & model_name)
    {
      Py_Initialize();

      bp::object main_module = bp::import("__main__");
      // Get a dict for the global namespace to exec further python code with
      bp::dict globals = bp::extract<bp::dict>(main_module.attr("__dict__"));

      // We need to link to the pinocchio PyWrap. We delegate the dynamic loading to the python interpreter.
      bp::object cpp_module( (bp::handle<>(bp::borrowed(PyImport_AddModule("libpinocchio_pywrap")))) );

      // That's it, you can exec your python script, starting with a model you
      // can update as you want.
      try
      {
// Boost 1.58
#if BOOST_VERSION / 100 % 1000 == 58
        // Avoid a segv with exec_file
        // See: https://github.com/boostorg/python/pull/15
        std::ifstream t(filename.c_str());
        std::stringstream buffer;
        buffer << t.rdbuf();
        bp::exec(buffer.str().c_str(), globals);
#else // default implementation
        bp::exec_file((bp::str)filename, globals);
#endif
      }
      catch (bp::error_already_set & e)
      {
        PyErr_PrintEx(0);
      }

      Model model;
      try
      {
        bp::object obj_model = globals[model_name];
        model = bp::extract<Model>(obj_model);
      }
      catch (bp::error_already_set & e)
      {
        PyErr_PrintEx(0);
      }

      // close the interpreter
      // cf. https://github.com/numpy/numpy/issues/8097
#if PY_MAJOR_VERSION < 3
      Py_Finalize();
#else

      PyObject * poMainModule = PyImport_AddModule("__main__");
      PyObject * poAttrList = PyObject_Dir(poMainModule);
      PyObject * poAttrIter = PyObject_GetIter(poAttrList);
      PyObject * poAttrName;

      while ((poAttrName = PyIter_Next(poAttrIter)) != NULL)
      {
        std::string oAttrName = PyUnicode_AS_DATA(poAttrName);

        // Make sure we don't delete any private objects.
        if (!boost::starts_with(oAttrName, "__") || !boost::ends_with(oAttrName, "__"))
        {
          PyObject * poAttr = PyObject_GetAttr(poMainModule, poAttrName);

          // Make sure we don't delete any module objects.
          if (poAttr && poAttr->ob_type != poMainModule->ob_type)
            PyObject_SetAttr(poMainModule, poAttrName, NULL);
          Py_DecRef(poAttr);
        }
        Py_DecRef(poAttrName);
      }
      Py_DecRef(poAttrIter);
      Py_DecRef(poAttrList);
#endif

      return model;
    }
  } // namespace python
} // namespace pinocchio
