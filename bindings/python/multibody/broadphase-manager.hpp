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
    : public boost::python::def_visitor< BroadPhaseManagerPythonVisitor<Derived> >
    {
    public:
      
      typedef BroadPhaseManagerTpl<Derived> Self;
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
             bp::return_value_policy<bp::copy_const_reference>())
        .def("getGeometryData",(GeometryData & (Self::*)())&Self::getGeometryData,
             bp::return_internal_reference<>())
        
        .def("update",(void (Self::*)(const bool))&Self::update,
             (bp::arg("compute_local_aabb") = true),
             "Update the manager from the current geometry positions and update the underlying FCL broad phase manager.")
        ;
      }
     
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        std::string derived_name = boost::typeindex::type_id<Derived>().pretty_name();
        boost::algorithm::replace_all(derived_name, "hpp::fcl::", "");
        const std::string class_name = "BroadPhaseManager_" + derived_name;
        
        bp::class_<Self, bp::bases<Derived> >(class_name.c_str(),
                                              "Broad phase manager.",
                                              bp::no_init)
        .def(BroadPhaseManagerPythonVisitor())
        ;
      }
      
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_broadphase_manager_hpp__
