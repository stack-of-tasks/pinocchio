//
// Copyright (c) 2024 CNRS
//

#ifndef __pinocchio_python_utils_path_hpp__
#define __pinocchio_python_utils_path_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    ///
    /// \brief python pathlib.Path | str -> C++ std::string
    ///
    std::string path(const bp::object & path);

    ///
    /// \brief python typing.List[pathlib.Path] | typing.List[str] -> C++ std::vector<std::string>
    ///
    std::vector<std::string> pathList(const bp::object & path_list);
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_path_hpp__
