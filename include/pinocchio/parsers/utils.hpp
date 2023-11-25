//
// Copyright (c) 2015 - 2016 CNRS
//

#ifndef __pinocchio_parsers_utils_hpp__
#define __pinocchio_parsers_utils_hpp__

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

namespace pinocchio
{
  ///
  /// \brief Supported model file extensions
  ///
  enum ModelFileExtensionType{
    UNKNOWN = 0,
    URDF
  };
  
  ///
  /// \brief Extract the type of the given model file according to its extension
  ///
  /// \param[in] filename The complete path to the model file.
  ///
  /// \return The type of the extension of the model file
  ///
  inline ModelFileExtensionType checkModelFileExtension(const std::string & filename)
  {
    const std::string extension = filename.substr(filename.find_last_of(".") + 1);
    
    if (extension == "urdf")
      return URDF;
    
    return UNKNOWN;
  }



  /**
   * @brief      Retrieve the path of the file whose path is given in URL-format.
   *             Currently convert from the following patterns : package:// or file://
   *
   * @param[in]  string          The path given in the url-format
   * @param[in]  package_dirs    A list of packages directories where to search for files 
   *                             if its pattern starts with package://
   *
   * @return     The path to the file (can be a relative or absolute path)
   */
   inline std::string retrieveResourcePath(const std::string & string,
                                           const std::vector<std::string> & package_dirs)
   {

    namespace bf = boost::filesystem;
    std::string result_path;

    const std::string separator("://");
    const std::size_t pos_separator = string.find(separator);
    bf::path string_path(string);

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
    else if (string_path.is_relative())
    {
      // handle the case where a relative mesh path is specified without using //package
      for (std::size_t i = 0; i < package_dirs.size(); ++i)
        {
          if ( bf::exists( bf::path(package_dirs[i] + "/" + string)))
          {
            result_path = std::string( package_dirs[i] + "/" + string);
            break;
          }
        }
    }
    else // return the entry string
    {
      result_path = string;
    } 

    return result_path;
   }

} // namespace pinocchio

#endif // __pinocchio_parsers_utils_hpp__
