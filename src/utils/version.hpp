//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_utils_version_hpp__
#define __se3_utils_version_hpp__

#include "pinocchio/macros.hpp"

#include <string>
#include <iostream>

namespace se3
{
  
  ///
  /// \brief Returns the current version of Pinocchio as a string using
  ///        the following standard:
  ///        PINOCCHIO_MINOR_VERSION.PINOCCHIO_MINOR_VERSION.PINOCCHIO_PATCH_VERSION
  ///
  std::string printVersion(const std::string & delimiter = ".")
  {
    std::ostringstream oss;
    oss
    << PINOCCHIO_MAJOR_VERSION << delimiter
    << PINOCCHIO_MINOR_VERSION << delimiter
    << PINOCCHIO_PATCH_VERSION;
    return oss.str();
  }
  
  ///
  /// \brief Checks if the current version of Pinocchio is at least the version provided
  ///        by the input arguments.
  ///
  /// \param[in] major_version Major version to check.
  /// \param[in] minor_version Minor version to check.
  /// \param[in] patch_version Patch version to check.
  ///
  /// \returns true if the current version of Pinocchio is greater than the version provided
  ///        by the input arguments.
  ///
  bool checkVersionAtLeast(unsigned int major_version,
                           unsigned int minor_version,
                           unsigned int patch_version)
  {
    return
    PINOCCHIO_MAJOR_VERSION > major_version
    || (PINOCCHIO_MAJOR_VERSION >= major_version
        && (PINOCCHIO_MINOR_VERSION > minor_version
            || (PINOCCHIO_MINOR_VERSION >= minor_version
                && PINOCCHIO_PATCH_VERSION >= patch_version)));
  }
}

#endif // __se3_utils_version_hpp__
