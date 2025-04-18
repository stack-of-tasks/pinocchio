//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_nb_utils_version_hpp__
#define __pinocchio_python_nb_utils_version_hpp__

#include "pinocchio/bindings/python_nb/fwd.hpp"

namespace pinocchio
{
  namespace python_nb
  {

    namespace nb = nanobind;
    using namespace nb::literals;

    void exposeVersion(nb::module_ &m);

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_nb_utils_version_hpp__