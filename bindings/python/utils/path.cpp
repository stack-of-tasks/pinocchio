//
// Copyright (c) 2024 CNRS
//

#include "pinocchio/bindings/python/utils/path.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    std::string path(const bp::object & path)
    {

      auto str_path = path;
      const bp::object Path = bp::import("pathlib").attr("Path");

      if (PyObject_IsInstance(str_path.ptr(), Path.ptr()))
        str_path = str_path.attr("__str__")();

      if (!PyObject_IsInstance(str_path.ptr(), reinterpret_cast<PyObject *>(&PyUnicode_Type)))
      {
        std::string what = bp::extract<std::string>(str_path.attr("__str__")())();
        throw std::invalid_argument(what + " is neither a Path nor a str.");
      }

      return bp::extract<std::string>(str_path);
    };
  } // namespace python
} // namespace pinocchio
