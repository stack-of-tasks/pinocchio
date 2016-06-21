#include <iostream>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/python/motion.hpp"
#include "pinocchio/python/inertia.hpp"
#include "pinocchio/python/model.hpp"

#include "pinocchio/multibody/parser/python.hpp"

namespace bp = boost::python;

namespace se3
{
  namespace python
  {
    Model buildModel (const std::string & filename, bool verbose)
    {
      Py_Initialize();
      // Get a dict for the global namespace to exec further python code with
      bp::object main_module = bp::import("__main__");
      bp::dict globals = bp::extract<bp::dict>(main_module.attr("__dict__"));

      // This will run all the needed expose() functions in python/python.cpp
      // for boost::python to know how to convert a PyObject * into an object
      bp::exec("import pinocchio as se3", globals);

      // Create a new Model, get a shared_ptr to it, and include this pointer
      // into the global namespace. See python/handler.hpp for more details.
      boost::shared_ptr<Model> model(new Model());
      bp::object obj(bp::handle<>(ModelPythonVisitor::convert( model )));
      globals["model"] = obj;

      // That's it, you can exec your python script, starting with a model you
      // can update as you want.
      try {
        bp::exec_file((bp::str)filename, globals);
      }
      catch (bp::error_already_set& e)
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
