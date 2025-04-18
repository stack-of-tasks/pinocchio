//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/bindings/python_nb/fwd.hpp"

namespace nb = nanobind;
using namespace pinocchio::python_nb;

NB_MODULE(PINOCCHIO_PYTHON_NB_MODULE_NAME, m)
{

  exposeVersion();
}
