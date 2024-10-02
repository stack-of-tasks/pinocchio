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

    std::vector<std::string> pathList(const bp::object & path_list)
    {
      if (!PyList_Check(path_list.ptr()))
      {
        std::string what = bp::extract<std::string>(path_list.attr("__str__")())();
        throw std::invalid_argument(what + " is not a list.");
      }

      // Retrieve the underlying list
      bp::object bp_obj(bp::handle<>(bp::borrowed(path_list.ptr())));
      bp::list bp_list(bp_obj);
      bp::ssize_t list_size = bp::len(bp_list);

      std::vector<std::string> path_vec;
      path_vec.reserve(list_size);
      // Check if all the elements contained in the current vector is of type T
      for (bp::ssize_t k = 0; k < list_size; ++k)
      {
        path_vec.push_back(path(bp_list[k]));
      }
      return path_vec;
    };
  } // namespace python
} // namespace pinocchio
