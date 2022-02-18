//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_algorithm_broadphase_manager_hpp__
#define __pinocchio_python_algorithm_broadphase_manager_hpp__

#include "pinocchio/multibody/broadphase-manager.hpp"

#include <boost/algorithm/string/replace.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename Derived>
    struct BroadPhaseManagerPythonVisitor
    : public bp::def_visitor< BroadPhaseManagerPythonVisitor<Derived> >
    {
    public:
      
      typedef BroadPhaseManagerTpl<Derived> Self;
      typedef typename Self::CollisionObjectVector CollisionObjectVector;
      typedef Derived Base;
      
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<const GeometryModel *, GeometryData *>(bp::args("self","geometry_model","geometry_data"),
                                                             "Default constructor")[bp::with_custodian_and_ward<1,2>(),bp::with_custodian_and_ward<1,3>()])
        .def(bp::init<const Self &>(bp::args("self","other"),
                                                             "Copy constructor")[bp::with_custodian_and_ward<1,2>()])
        
        .def("getGeometryModel",&Self::getGeometryModel,
             bp::arg("self"),
             bp::return_value_policy<bp::copy_const_reference>())
        .def("getGeometryData",(GeometryData & (Self::*)())&Self::getGeometryData,
             bp::arg("self"),
             bp::return_internal_reference<>())
        .def("getCollisionObjects",(CollisionObjectVector & (Self::*)())&Self::getCollisionObjects,
             bp::arg("self"),
             bp::return_internal_reference<>())
        
        .def("check", (bool (Self::*)() const)&Self::check,
             bp::arg("self"),
             "Check whether the base broad phase manager is aligned with the current collision_objects.")
        .def("check", (bool (Self::*)(CollisionCallBackBase *) const)&Self::check,
             bp::args("self","callback"),
             "Check whether the callback is inline with *this.")
        ;
      }
     
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        std::string derived_name = boost::typeindex::type_id<Derived>().pretty_name();
        boost::algorithm::replace_all(derived_name, "hpp::fcl::", "");
        const std::string class_name = "BroadPhaseManager_" + derived_name;
        
        bp::class_<Self, bp::bases<Base>> registered_class(class_name.c_str(),
                                                           "Broad phase manager.",
                                                           bp::no_init);
        registered_class.def(BroadPhaseManagerPythonVisitor());
        
        const bp::type_info base_info = bp::type_id<hpp::fcl::BroadPhaseCollisionManager>();
        const bp::converter::registration* base_reg = bp::converter::registry::query(base_info);
        bp::object base_class_obj(bp::handle<>(bp::borrowed(base_reg->get_class_object())));

        const bp::str method_name("update");
        PyObject* const name_space = base_class_obj.ptr();
        
        bp::handle<> dict;
        
#if PY_VERSION_HEX < 0x03000000
        // Old-style class gone in Python 3
        if (PyClass_Check(name_space))
            dict = bp::handle<>(bp::borrowed(((PyClassObject*)name_space)->cl_dict));
        else
#endif
        if (PyType_Check(name_space))
            dict = bp::handle<>(bp::borrowed(((PyTypeObject*)name_space)->tp_dict));
        else
            dict = bp::handle<>(PyObject_GetAttrString(name_space, const_cast<char*>("__dict__")));

        if (dict == 0)
          bp::throw_error_already_set();
        
        bp::handle<> existing(bp::allow_null(::PyObject_GetItem(dict.get(), method_name.ptr())));
        PyErr_Clear();

        if (existing)
        {
          bp::object base_methods_as_object(bp::handle<>(static_cast<PyObject*>(existing.get())));
          bp::objects::add_to_namespace(registered_class,"update",base_methods_as_object,"");//base_methods->doc());
          
          bp::objects::add_to_namespace(registered_class,"update",
                                        bp::make_function((void (Self::*)(const bool))&Self::update,
                                                          bp::default_call_policies(),
                                                          (bp::arg("self"),bp::arg("compute_local_aabb"))),
                                        "Update the manager from the current geometry positions and update the underlying FCL broad phase manager.");
          
        }
      }
      
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_broadphase_manager_hpp__
