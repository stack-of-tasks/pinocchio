//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_utils_version_hpp__
#define __pinocchio_utils_version_hpp__

#include "pinocchio/config.hpp"

#include <string>
#include <sstream>

namespace pinocchio
{
  
  ///
  /// \brief Returns the current version of Pinocchio as a string using
  ///        the following standard:
  ///        PINOCCHIO_MINOR_VERSION.PINOCCHIO_MINOR_VERSION.PINOCCHIO_PATCH_VERSION
  ///
  inline std::string printVersion(const std::string & delimiter = ".")
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
  inline bool checkVersionAtLeast(unsigned int major_version,
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

#endif // __pinocchio_utils_version_hpp__
