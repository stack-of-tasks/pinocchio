//
// Copyright (c) 2016-2021 CNRS INRIA
//

#include <cstdlib>
#include <boost/filesystem.hpp>
#include "pinocchio/utils/file-explorer.hpp"

#ifdef _WIN32
const char delimiter = ';';
#else
const char delimiter = ':';
#endif

namespace fs = boost::filesystem;
namespace pinocchio
{

  void extractPathFromEnvVar(const std::string & env_var_name,
                             std::vector<std::string> & list_of_paths)
  {
    const char * env_var_value = std::getenv(env_var_name.c_str());
    
    if(env_var_value != NULL)
    {
      std::string policyStr(env_var_value);
      // Add a separator at the end so that last path is also retrieved
      policyStr += delimiter;
      size_t lastOffset = 0;
      while(true)
      {
        size_t offset = policyStr.find_first_of(delimiter, lastOffset);
        if (offset < policyStr.size())
          list_of_paths.push_back(policyStr.substr(lastOffset, offset - lastOffset));
        if (offset == std::string::npos)
          break;
        else
          lastOffset = offset + 1; // add one to skip the delimiter
      }
    }
  }

  std::vector<std::string> extractPathFromEnvVar(const std::string & env_var_name)
  {
    std::vector<std::string> list_of_paths;
    extractPathFromEnvVar(env_var_name,list_of_paths);
    return list_of_paths;
  }

  std::vector<std::string> rosPaths()
  {
    std::vector<std::string> raw_list_of_paths;
    extractPathFromEnvVar("ROS_PACKAGE_PATH", raw_list_of_paths);
    extractPathFromEnvVar("AMENT_PREFIX_PATH", raw_list_of_paths);

    // Work-around for https://github.com/stack-of-tasks/pinocchio/issues/1463
    // To support ROS devel/isolated spaces, we also need to look one package above the package.xml:
    fs::path path;
    std::vector<std::string> list_of_paths;
    for (const auto& path_string : raw_list_of_paths) {
      list_of_paths.push_back(path_string);
      path = fs::path(path_string);
      if (fs::exists(path / "package.xml")) {
        list_of_paths.push_back(fs::path(path / "..").string());
      }
    }
    return list_of_paths;
  }

}
