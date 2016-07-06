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

#ifndef __se3_pythonparser_hpp__
#define __se3_pythonparser_hpp__

#include <boost/python.hpp>

#include "pinocchio/multibody/model.hpp"

namespace se3
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
    Model buildModel(const std::string & filename, const std::string & var_name = "model", bool verbose = false) throw (boost::python::error_already_set);
    
  } // namespace python
  
} // namespace se3

#endif // ifndef __se3_pythonparser_hpp__
