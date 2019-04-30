//
// Copyright (c) 2016 CNRS
//

#ifndef __pinocchio_pythonparser_hpp__
#define __pinocchio_pythonparser_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/model.hpp"

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
    /// \input verbose Verbosity mode.
    ///
    /// \returns The model constructed by the Python script.
    ///
    // TODO: look inside the context of Python and find an occurence of object Model
    Model buildModel(const std::string & filename, const std::string & var_name = "model", bool verbose = false);
    
  } // namespace python
  
} // namespace pinocchio

#endif // ifndef __pinocchio_pythonparser_hpp__
