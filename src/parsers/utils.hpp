//
// Copyright (c) 2015 - 2016 CNRS
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

#ifndef __se3_parsers_utils_hpp__
#define __se3_parsers_utils_hpp__

#include <iostream>
#include <limits>
#include <sstream>
// #include <stdexcept>

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "pinocchio/utils/file-explorer.hpp"
#include <boost/filesystem.hpp>

#include <exception>

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
  inline ModelFileExtensionType checkModelFileExtension (const std::string & filename)
  {
    const std::string extension = filename.substr(filename.find_last_of(".") + 1);
    
    if (extension == "urdf")
      return URDF;
    else if (extension == "lua")
      return LUA;
    
    return UNKNOWN;
  }



  /**
   * @brief      Retrieve the path of the file whose path is given in an url-format.
   *             Currently convert from the folliwing patterns : package:// or file://
   *
   * @param[in]  string          The path given in the url-format
   * @param[in]  package_dirs    A list of packages directories where to search for files 
   *                             if its pattern starts with package://
   *
   * @return     The path to the file (can be a relative or absolute path)
   */
   inline std::string retrieveResourcePath(const std::string & string,
                                           const std::vector<std::string> & package_dirs) throw (std::invalid_argument)
   {

    namespace bf = boost::filesystem;
    std::string result_path;

    const std::string separator("://");
    const std::size_t pos_separator = string.find(separator);

    if (pos_separator != std::string::npos)
    {
      std::string scheme = string.substr(0, pos_separator);
      std::string path = string.substr(pos_separator+3, std::string::npos);

      if(scheme == "package")
      {
        // if exists p1/string, path = p1/string,
        // else if exists p2/string, path = p2/string
        // else return an empty string that may provoke an error in loadPolyhedronFromResource()

        // concatenate package_path with filename
        for (std::size_t i = 0; i < package_dirs.size(); ++i)
        {
          if ( bf::exists( bf::path(package_dirs[i] + "/" + path)))
          {
            result_path = std::string( package_dirs[i] + "/" + path );
            break;
          }
        }
      }
      else if (scheme == "file")
      {
        result_path = path;
      }
      else
      {
        const std::string exception_message ("Schemes of form" + scheme + "are not handled");
        throw std::invalid_argument(exception_message);
      }
    }
    else // return the entry string
    {
      result_path = string;
    } 

    return result_path;
   }

} // namespace se3

#endif // __se3_parsers_utils_hpp__
