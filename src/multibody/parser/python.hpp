#ifndef __se3_pythonparser_hpp__
#define __se3_pythonparser_hpp__

#include "pinocchio/multibody/model.hpp"

namespace se3
{
  namespace python
  {
    Model buildModel (const std::string & filename, bool verbose = false);
  } // namespace python
} // namespace se3

#endif // ifndef __se3_pythonparser_hpp__
