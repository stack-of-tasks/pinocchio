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

#include <iostream>

namespace se3
{
  ///
  /// \brief Supported model file extensions
  ///
  enum ModelFileExtensionType{
    UNKNOWN = 0,
    URDF,
    LUA
  };
  
  ///
  /// \brief Extract the type of the given model file according to its extension
  ///
  /// \param[in] filemane The complete path to the model file.
  ///
  /// \return The type of the extension of the model file
  ///
  ModelFileExtensionType checkModelFileExtension (const std::string & filename)
  {
    const std::string extension = filename.substr(filename.find_last_of(".") + 1);
    
    if (extension == "urdf")
      return URDF;
    else if (extension == "lua")
      return LUA;
    
    return UNKNOWN;
  }
} // namespace se3
