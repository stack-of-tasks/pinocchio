//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/bindings/python_nb/fwd.hpp"
#include "pinocchio/utils/version.hpp"


namespace pinocchio
{
  namespace python_nb
  {

    namespace nb = nanobind;
    using namespace nb::literals;

    void exposeVersion(nb::module_ &m)
    {
      m.attr("PINOCCHIO_MAJOR_VERSION") = PINOCCHIO_MAJOR_VERSION;
      m.attr("PINOCCHIO_MINOR_VERSION") = PINOCCHIO_MINOR_VERSION;
      m.attr("PINOCCHIO_PATCH_VERSION") = PINOCCHIO_PATCH_VERSION;

      m.def("printVersion", printVersion, "delimiter"_a = ".",
      "Returns the current version of Pinocchio as a string.\n"
        "The user may specify the delimiter between the different semantic numbers.");

      m.def("checkVersionAtLeast", &checkVersionAtLeast, "major"_a, "minor"_a, "patch"_a,
      "Checks if the current version of Pinocchio is at least"
        " the version provided by the input arguments.");
    }

  } // namespace python_nb
} // namespace pinocchio