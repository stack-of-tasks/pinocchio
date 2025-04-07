//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_utils_model_checker_hpp__
#define __pinocchio_python_utils_model_checker_hpp__

#include <Python.h>
#include <string>

#include "pinocchio/algorithm/check-model.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {

    // Checker Policy to make a safe version of python function, concerning joint mimic.
    // This will throw a runtime error in case algorithm does not allow joint mimic in the model.
    template<class Policy = boost::python::default_call_policies>
    struct mimic_not_supported_function : Policy
    {
      mimic_not_supported_function(size_t model_idx_)
      : Policy()
      , model_idx(model_idx_)
      {
      }

      template<class ArgumentPackage>
      bool precall(ArgumentPackage const & args) const
      {
        // Convert the object to a tuple
        boost::python::tuple py_args = boost::python::extract<boost::python::tuple>(args);

        context::Model m = boost::python::extract<context::Model>(py_args[model_idx]);
        if (!m.check(MimicChecker()))
        {
          PyErr_SetString(PyExc_RuntimeError, m_error_message.c_str());
          return false;
        }
        else
          return static_cast<const Policy *>(this)->precall(args);
      }

    protected:
      static const std::string m_error_message;
      // index of the pinocchio in the function arguments. Need it to avoid a loop
      const size_t model_idx;
    };

    template<class Policy>
    const std::string mimic_not_supported_function<Policy>::m_error_message =
      "This algorithm does not support Joint Mimic type in the model.";
  } // namespace python
} // namespace pinocchio
#endif // model_checker
