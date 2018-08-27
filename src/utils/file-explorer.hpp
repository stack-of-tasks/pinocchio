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

#ifndef __se3_file_explorer_hpp__
#define __se3_file_explorer_hpp__

#include <string>
#include <iostream>
#include <vector>

namespace se3
{

  /**
   * @brief      Parse an environment variable if exists and extract paths according to the delimiter.
   *
   * @param[in]  env_var_name The name of the environment variable.
   * @param[in]  delimiter The delimiter between two consecutive paths.
   *
   * @return The vector of paths extracted from the environment variable value.
   */
  inline std::vector<std::string> extractPathFromEnvVar(const std::string & env_var_name, const std::string & delimiter = ":")
  {
    const char * env_var_value = std::getenv(env_var_name.c_str());
    std::vector<std::string> env_var_paths;
    
    if (env_var_value != NULL)
    {
      std::string policyStr (env_var_value);
      // Add a separator at the end so that last path is also retrieved
      policyStr += std::string (":");
      size_t lastOffset = 0;
      
      while(true)
      {
        size_t offset = policyStr.find_first_of(delimiter, lastOffset);
        if (offset < policyStr.size())
          env_var_paths.push_back(policyStr.substr(lastOffset, offset - lastOffset));
        if (offset == std::string::npos)
          break;
        else
          lastOffset = offset + 1; // add one to skip the delimiter
      }
    }
    
    return env_var_paths;
  }


  /**
   * @brief      Parse the environment variable ROS_PACKAGE_PATH and extract paths
   *
   * @return     The vector of paths extracted from the environment variable ROS_PACKAGE_PATH
   */
  inline std::vector<std::string> rosPaths()
  {
    return extractPathFromEnvVar("ROS_PACKAGE_PATH");
  }

}

#endif // __se3_file_explorer_hpp__
