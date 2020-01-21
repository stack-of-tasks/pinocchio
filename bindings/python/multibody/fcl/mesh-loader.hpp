//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_python_fcl_mesh_loader_hpp__
#define __pinocchio_python_fcl_mesh_loader_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/mesh_loader/loader.h>

#include <boost/python.hpp>
#include <boost/python/type_id.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace fcl
    {
      
      namespace bp = boost::python;
      
      template<typename MeshLoader>
      struct MeshLoaderPythonVisitor
      : public bp::def_visitor< MeshLoaderPythonVisitor<MeshLoader> >
      {
        typedef boost::shared_ptr<MeshLoader> MeshLoaderPtr_t;
        
        template <typename T>
        static boost::shared_ptr<T> create()
        {
          return boost::shared_ptr<T>(new T);
        }

        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def(bp::init<>(bp::arg("self"),"Default constructor"))
          .def("create",&MeshLoaderPythonVisitor::create<MeshLoader>,"Create a new object.")
          .staticmethod("create")
          ;
        }
        
        static void expose(const std::string & doc = "")
        {
          static const bp::type_info info = bp::type_id<MeshLoader>();
          static const std::string class_name = info.name();
          static const std::string class_name_without_namespace = class_name.substr(class_name.find_last_of(':')+1);
          
          bp::class_<MeshLoader, MeshLoaderPtr_t>
            (class_name_without_namespace.c_str(),
             doc.c_str(),
             bp::no_init)
            .def(MeshLoaderPythonVisitor())
          ;
        }
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_mesh_loader_hpp__
