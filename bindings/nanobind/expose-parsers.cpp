// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/filesystem.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeConsoleBridge(nb::module_ m);
void exposeURDFModel(nb::module_ m);
void exposeURDFGeometry(nb::module_ m);
void exposeSRDF(nb::module_ m);
void exposeMJCF(nb::module_ m);
#ifdef PINOCCHIO_WITH_SDFORMAT // corresponding files not compiled in CMake otherwise
void exposeSDFModel(nb::module_ m);
void exposeSDFGeometry(nb::module_ m);
#endif

void exposeParsers(nb::module_ m)
{
  // URDF
  {
    exposeConsoleBridge(m);
    exposeURDFModel(m);
    exposeURDFGeometry(m);
  }

  exposeSRDF(m);
  exposeMJCF(m);

#ifdef PINOCCHIO_WITH_SDFORMAT
  // SDF
  {
    exposeSDFModel(m);
    exposeSDFGeometry(m);
  }
#endif

  nb::bind_vector<std::vector<std::filesystem::path>>(m, "StdVec_Path");
}

PINOCCHIO_PYTHON_NAMESPACE_END
