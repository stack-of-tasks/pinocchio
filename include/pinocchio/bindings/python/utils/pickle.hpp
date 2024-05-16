//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_python_utils_pickle_hpp__
#define __pinocchio_python_utils_pickle_hpp__

#include <pinocchio/bindings/python/fwd.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Derived>
    struct PickleFromStringSerialization : bp::pickle_suite
    {
      static bp::tuple getinitargs(const Derived &)
      {
        return bp::make_tuple();
      }

      static bp::tuple getstate(const Derived & obj)
      {
        const std::string str(obj.saveToString());
        return bp::make_tuple(bp::str(str));
      }

      static void setstate(Derived & obj, bp::tuple tup)
      {
        if (bp::len(tup) == 0 || bp::len(tup) > 1)
        {
          throw eigenpy::Exception(
            "Pickle was not able to reconstruct the object from the loaded data.\n"
            "The pickle data structure contains too many elements.");
        }

        bp::object py_obj = tup[0];
        boost::python::extract<std::string> obj_as_string(py_obj.ptr());
        if (obj_as_string.check())
        {
          const std::string str = obj_as_string;
          obj.loadFromString(str);
        }
        else
        {
          throw eigenpy::Exception(
            "Pickle was not able to reconstruct the model from the loaded data.\n"
            "The entry is not a string.");
        }
      }

      static bool getstate_manages_dict()
      {
        return true;
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_pickle_hpp__
