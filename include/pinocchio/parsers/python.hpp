//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_parser_python_hpp__
#define __pinocchio_parser_python_hpp__

#include "pinocchio/python_parser/config.hpp"
#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{
  namespace python
  {
    /// \brief Load a model from a Python script.
    ///
    /// This function raises a Python error in case of inconsistency in the Python code.
    ///
    /// \input filename The full path to the model file.
    /// \input var_name Name of the Python variable which contains the model in the script.
    ///
    /// \returns The model constructed by the Python script.
    ///
    // TODO: look inside the context of Python and find an occurence of object Model
    PINOCCHIO_PYTHON_PARSER_DLLAPI
    Model buildModel(const std::string & filename, const std::string & var_name = "model");

  } // namespace python

} // namespace pinocchio

#endif // ifndef __pinocchio_parser_python_hpp__
