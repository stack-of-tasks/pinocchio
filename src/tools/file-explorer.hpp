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

#include "boost/filesystem.hpp"

namespace se3
{

    /**
     * @brief      Parse env variable ROS_PACKAGE_PATH to extract paths
     *
     * @return     The different paths in ROS_PACKAGE_PATH
     */
    inline std::vector < std::string > getRosPackagePaths()
    {
        std::vector<std::string> results;

        std::string delimiter = ":";
        std::string policyStr = std::getenv("ROS_PACKAGE_PATH");
        size_t lastOffset = 0;

        while(true)
        {
            size_t offset = policyStr.find_first_of(delimiter, lastOffset);
            if (offset < policyStr.size())
              results.push_back(policyStr.substr(lastOffset, offset - lastOffset));
            if (offset == std::string::npos)
                break;
            else
                lastOffset = offset + 1; // add one to skip the delimiter
        }

        return results;
    }

}

#endif // __se3_file_explorer_hpp__
