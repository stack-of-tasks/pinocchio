//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_collision_geometry_functors_hpp__
#define __pinocchio_python_collision_geometry_functors_hpp__

#include "pinocchio/bindings/python/utils/registration.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename GeometryFunctor>
    struct GeometryFunctorPythonVisitor
    : public boost::python::def_visitor<GeometryFunctorPythonVisitor<GeometryFunctor>>
    {

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        const std::string class_name = bp::type_id<GeometryFunctor>().name();

        cl.def(bp::init<const GeometryObject &, const GeometryObject &>(
                 bp::args("self", "geometry_object1", "geometry_object2"),
                 (std::string("Constructor of a ") + class_name).c_str()))
          .def(
            "run", &GeometryFunctor::run, bp::args("self", "tf1", "tf2", "request", "result"),
            "Call the function and return the result")

          .def(
            "getGeometryObject1", &GeometryFunctor::getGeometryObject1,
            bp::return_value_policy<bp::copy_const_reference>())
          .def(
            "getGeometryObject2", &GeometryFunctor::getGeometryObject2,
            bp::return_value_policy<bp::copy_const_reference>())

          .def("print", &print);
      }

      static void print(const GeometryFunctor & self)
      {
        std::cout << "address #1: " << &self.getGeometryObject1() << std::endl;
        std::cout << "address #2: " << &self.getGeometryObject2() << std::endl;
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_collision_geometry_functors_hpp__
