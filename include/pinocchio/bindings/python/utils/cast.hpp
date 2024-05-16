//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_cast_hpp__
#define __pinocchio_python_utils_cast_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

#include <eigenpy/registration.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    ///
    /// \brief Add the Python method cast
    ///
    template<class C, typename NewScalar = PINOCCHIO_PYTHON_SCALAR_TYPE_DEFAULT>
    struct CastVisitor : public bp::def_visitor<CastVisitor<C, NewScalar>>
    {
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def("cast", &C::template cast<NewScalar>, "Returns a cast of *this.");
      }
    };

    template<class _From, class _To>
    struct ExposeConstructorByCastVisitor
    : public bp::def_visitor<ExposeConstructorByCastVisitor<_From, _To>>
    {
      template<class PyClass>
      void visit(PyClass & /*cl*/) const
      {
        expose_constructor<_From, _To>();
        expose_constructor<_To, _From>();
      }

    protected:
      template<typename From, typename To>
      static void expose_constructor()
      {
        if (eigenpy::check_registration<From>() && eigenpy::check_registration<To>())
        {
          const bp::type_info to_info = bp::type_id<To>();
          const bp::converter::registration * to_reg = bp::converter::registry::query(to_info);
          bp::object to_class_obj(bp::handle<>(bp::borrowed(to_reg->get_class_object())));
          const std::string to_module_name =
            bp::extract<std::string>(to_class_obj.attr("__module__"));
          const std::string to_class_name = bp::extract<std::string>(to_class_obj.attr("__name__"));

          const bp::type_info from_info = bp::type_id<From>();
          const bp::converter::registration * from_reg = bp::converter::registry::query(from_info);
          bp::object from_class_obj(bp::handle<>(bp::borrowed(from_reg->get_class_object())));
          const std::string from_module_name =
            bp::extract<std::string>(from_class_obj.attr("__module__"));
          const std::string from_class_name =
            bp::extract<std::string>(from_class_obj.attr("__name__"));

          const std::string to_full_class_name = to_module_name + "." + to_class_name;
          const std::string from_full_class_name = from_module_name + "." + from_class_name;
          std::stringstream to_doc;
          to_doc << "Copy constructor from " << from_full_class_name;
          to_doc << " -> " << to_full_class_name;
          bp::objects::add_to_namespace(
            to_class_obj, "__init__",
            bp::make_constructor(
              &ExposeConstructorByCastVisitor::constructor<From, To>, bp::default_call_policies(),
              bp::arg("clone")),
            to_doc.str().c_str());
        }
      }

      template<typename From, typename To>
      static To * constructor(const From & clone)
      {
        typedef typename To::Scalar NewScalar;
        return new To(clone.template cast<NewScalar>());
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_cast_hpp__
