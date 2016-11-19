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
//
// Code adapted from https://bitbucket.org/rbdl/rbdl

#include <iostream>

#include "pinocchio/multibody/model.hpp"

#ifdef WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
#endif

#ifdef WITH_LUA5
  #include "pinocchio/parsers/lua.hpp"
#endif

#include "pinocchio/parsers/utils.hpp"

using namespace std;

void usage (const char* application_name) {
  cerr << "Usage: " << application_name << " [-v] [-h] <model.extension>" << endl;
  cerr << "  -v | --verbose            default parser verbosity" << endl;
  cerr << "  -h | --help               print this help" << endl;
  exit (1);
}

int main(int argc, char *argv[])
{
  if (argc < 2 || argc > 4) {
    usage(argv[0]);
  }
  
  std::string filename;
  
  bool verbose = false;
  
  for (int i = 1; i < argc; i++) {
    if (string(argv[i]) == "-v" || string (argv[i]) == "--verbose")
      verbose = true;
    else if (string(argv[i]) == "-h" || string (argv[i]) == "--help")
      usage(argv[0]);
    else
      filename = argv[i];
  }
  
  // Check extension of the file
  se3::ModelFileExtensionType extension_type = se3::checkModelFileExtension(filename);
  se3::Model model;
  
  switch(extension_type)
  {
    case se3::URDF:
#ifdef WITH_URDFDOM
      se3::urdf::buildModel(filename,model,verbose);
#else
      std::cerr << "It seems that the URDFDOM module has not been found during the Cmake process." << std::endl;
#endif
      break;
    case se3::LUA:
#ifdef WITH_LUA5
      model = se3::lua::buildModel(filename, false, verbose);
#else
      std::cerr << "It seems that the LUA module has not been found during the Cmake process." << std::endl;
#endif
      break;
    case se3::UNKNOWN:
      std::cerr << "Unknown extension of " << filename << std::endl;
      return -1;
      break;
  }
  return 0;
}
