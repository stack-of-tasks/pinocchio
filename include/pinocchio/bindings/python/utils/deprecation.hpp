//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_deprecation_hpp__
#define __pinocchio_python_utils_deprecation_hpp__

#include <Python.h>
#include <boost/python.hpp>
#include <string>

namespace pinocchio
{
  namespace python
  {
    template<class Policy = boost::python::default_call_policies>
    struct deprecated_warning_policy : Policy
    {
      deprecated_warning_policy(const std::string & warning_message = "")
      : Policy()
      , m_warning_message(warning_message)
      {}
      
      template <class ArgumentPackage>
      bool precall(ArgumentPackage const & args) const
      {
        PyErr_WarnEx(PyExc_UserWarning,m_warning_message.c_str(),1);
        return static_cast<const Policy*>(this)->precall(args);
      }
      
      typedef typename Policy::result_converter result_converter;
      typedef typename Policy::argument_package argument_package;
        
    protected:
      
      const std::string m_warning_message;
    };
  
    template<class Policy = boost::python::default_call_policies>
    struct deprecated_member : deprecated_warning_policy<Policy>
    {
      deprecated_member(const std::string & warning_message
                        = "This class member has been marked as deprecated and will be removed in a future release.")
      : deprecated_warning_policy<Policy>(warning_message)
      {}
    };
  
    template<class Policy = boost::python::default_call_policies>
    struct deprecated_function : deprecated_warning_policy<Policy>
    {
      deprecated_function(const std::string & warning_message
                        = "This function has been marked as deprecated and will be removed in a future release.")
      : deprecated_warning_policy<Policy>(warning_message)
      {}
    };
  }
}

#endif // ifndef __pinocchio_python_utils_deprecation_hpp__
