//
// Copyright (c) 2015 CNRS
//
//
// Code adapted from https://bitbucket.org/rbdl/rbdl

#include <iostream>

#include "pinocchio/multibody/model.hpp"

#ifdef PINOCCHIO_WITH_URDFDOM
  #include "pinocchio/parsers/urdf.hpp"
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
  pinocchio::ModelFileExtensionType extension_type = pinocchio::checkModelFileExtension(filename);
  pinocchio::Model model;
  
  switch(extension_type)
  {
    case pinocchio::URDF:
#ifdef PINOCCHIO_WITH_URDFDOM
      pinocchio::urdf::buildModel(filename,model,verbose);
#else
      std::cerr << "It seems that the URDFDOM module has not been found during the Cmake process." << std::endl;
#endif
      break;
    case pinocchio::UNKNOWN:
      std::cerr << "Unknown extension of " << filename << std::endl;
      return -1;
      break;
  }
  return 0;
}
