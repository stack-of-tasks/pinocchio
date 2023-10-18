//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_python_parser_python_hpp__
#define __pinocchio_python_parser_python_hpp__

#include "pinocchio/multibody/model.hpp"

#include <boost/python.hpp>

#if defined _WIN32
# ifdef pinocchio_pywrap_EXPORTS
#   define PINOCCHIO_PYWRAP_DLLAPI __declspec(dllexport)
# else
#   define PINOCCHIO_PYWRAP_DLLAPI __declspec(dllimport)
# endif // pinocchio_pywrap_EXPORTS
#else
# define PINOCCHIO_PYWRAP_DLLAPI
#endif // _WIN32

namespace pinocchio
{
  namespace python
  {
    /// \brief Load a model from a Python script.
    ///
    /// This function raises a Python error in case of incistency in the Python code.
    ///
    /// \input filename The full path to the model file.
    /// \input var_name Name of the Python variable which contains the model in the script.
    ///
    /// \returns The model constructed by the Python script.
    ///
    // TODO: look inside the context of Python and find an occurence of object Model
    PINOCCHIO_PYWRAP_DLLAPI
    Model buildModel(const std::string & filename,
                     const std::string & var_name = "model");
    
    ///
    /// \copydoc pinocchio::python::buildModel(const std::string &, const std::string &)
    ///
    PINOCCHIO_DEPRECATED
    Model buildModel(const std::string & filename,
                     const std::string & var_name,
                     const bool /*verbose*/)
    {
      return buildModel(filename,var_name);
    }
    
  } // namespace python
  
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parser_python_hpp__
