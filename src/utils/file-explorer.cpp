//
// Copyright (c) 2016-2021 CNRS INRIA
//

#include <cstdlib>
#include <boost/filesystem.hpp>
#include "pinocchio/utils/file-explorer.hpp"

namespace fs = boost::filesystem;
namespace pinocchio
{

  void extractPathFromEnvVar(const std::string & env_var_name,
                             std::vector<std::string> & list_of_paths,
                             const std::string & delimiter)
  {
    const char * env_var_value = std::getenv(env_var_name.c_str());
    
    if(env_var_value != NULL)
    {
      std::string policyStr(env_var_value);
      // Add a separator at the end so that last path is also retrieved
      policyStr += std::string(":");
      size_t lastOffset = 0;
      std::string path_string;
      path_string.reserve(500);
      fs::path path;
      
      while(true)
      {
        size_t offset = policyStr.find_first_of(delimiter, lastOffset);
        if (offset < policyStr.size()) {
          path_string = policyStr.substr(lastOffset, offset - lastOffset);
          path = fs::path(path_string);
          list_of_paths.push_back(path_string);
          // Work-around for https://github.com/stack-of-tasks/pinocchio/issues/1463
          // To support ROS devel/isolated spaces, we also need to look one package above the package.xml:
          if (fs::exists(path / "package.xml")) {
            list_of_paths.push_back(fs::path(path / "..").string());
          }
        }
        if (offset == std::string::npos)
          break;
        else
          lastOffset = offset + 1; // add one to skip the delimiter
      }
    }
  }

  std::vector<std::string> extractPathFromEnvVar(const std::string & env_var_name,
                                                 const std::string & delimiter)
  {
    std::vector<std::string> list_of_paths;
    extractPathFromEnvVar(env_var_name,list_of_paths,delimiter);
    return list_of_paths;
  }

  std::vector<std::string> rosPaths()
  {
    std::vector<std::string> list_of_paths;
    extractPathFromEnvVar("ROS_PACKAGE_PATH",list_of_paths);
    extractPathFromEnvVar("AMENT_PREFIX_PATH",list_of_paths);
    
    return list_of_paths;
  }

}
