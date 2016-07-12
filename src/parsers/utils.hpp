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

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "pinocchio/tools/file-explorer.hpp"
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
   * @brief      Transform a package://.. mesh path to an absolute path, searching for a valid file 
   *             in a list of package directories
   *
   * @param[in]  urdf_mesh_path  The path given in the urdf file
   * @param[in]  package_dirs    A list of packages directories where to search for meshes
   *
   * @return     The absolute path to the mesh file
   */
   inline std::string convertURDFMeshPathToAbsolutePath(const std::string & urdf_mesh_path,
                                                         const std::vector<std::string> & package_dirs)
   {
    // if exists p1/mesh, absolutePath = p1/mesh,
    // else if exists p2/mesh, absolutePath = p2/mesh
    // else return an empty string that will provoke an error in loadPolyhedronFromResource()
    namespace bf = boost::filesystem;

    std::string absolutePath;
    // concatenate package_path with mesh filename
    for (std::size_t i = 0; i < package_dirs.size(); ++i)
    {
      if ( bf::exists( bf::path(package_dirs[i] +  urdf_mesh_path.substr(9, urdf_mesh_path.size()))))
      {
          absolutePath = std::string( package_dirs[i] + urdf_mesh_path.substr(9, urdf_mesh_path.size())
                                       );
        break;
      }
    }
    return absolutePath;
   }

} // namespace se3

#endif // __se3_parsers_utils_hpp__