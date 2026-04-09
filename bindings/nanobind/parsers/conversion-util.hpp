// Copyright (c) 2026 INRIA

#ifdef PINOCCHIO_WITH_COLLISION
  #include <coal/mesh_loader/loader.h>
#endif
#include "pinocchio/bindings/python-nb/fwd.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/variant.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace fs = std::filesystem;
using PyPkgDirArg = std::variant<fs::path, std::vector<fs::path>>;

inline std::vector<std::string> pkgDirsFromArg(const PyPkgDirArg & py_pkg_dirs)
{
  if (py_pkg_dirs.index() == 0)
  {
    return {std::get<fs::path>(py_pkg_dirs).string()};
  }
  else
  {
    auto path_list = std::get<std::vector<fs::path>>(py_pkg_dirs);
    std::vector<std::string> out;
    for (auto && p : path_list)
    {
      out.emplace_back(p.string());
    }
    return out;
  }
}

inline ::coal::MeshLoaderPtr meshLoaderFromObject(nb::object py_mesh_loader)
{
  if (py_mesh_loader.is_none())
    return nullptr;
#ifdef PINOCCHIO_WITH_COLLISION
  return nb::cast<::coal::MeshLoaderPtr>(py_mesh_loader);
#else
  PyErr_WarnEx(
    PyExc_UserWarning, "Mesh loader is ignored because Pinocchio is not built with coal", 1);
  return nullptr;
#endif
}

PINOCCHIO_PYTHON_NAMESPACE_END
