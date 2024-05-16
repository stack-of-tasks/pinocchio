//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_python_utils_registration_hpp__
#define __pinocchio_python_utils_registration_hpp__

#include <eigenpy/registration.hpp>

namespace pinocchio
{
  namespace python
  {

    template<typename T>
    inline bool register_symbolic_link_to_registered_type()
    {
      namespace bp = boost::python;
      if (eigenpy::check_registration<T>())
      {
        const bp::type_info info = bp::type_id<T>();
        const bp::converter::registration * reg = bp::converter::registry::query(info);
        bp::handle<> class_obj(bp::borrowed(reg->get_class_object()));
        bp::scope().attr(reg->get_class_object()->tp_name) = bp::object(class_obj);
        return true;
      }

      return false;
    }

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_registration_hpp__
