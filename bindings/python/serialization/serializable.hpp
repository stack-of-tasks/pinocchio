//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_python_serialization_serializable_hpp__
#define __pinocchio_python_serialization_serializable_hpp__

#include <boost/python.hpp>

#include "pinocchio/serialization/serializable.hpp"

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename Derived>
    struct SerializableVisitor
    : public bp::def_visitor< SerializableVisitor<Derived> >
    {
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def("saveToText",&Derived::saveToText,
             bp::arg("filename"),"Saves *this inside a text file.")
        .def("loadFromText",&Derived::loadFromText,
             bp::arg("filename"),"Loads *this from a text file.")
        .def("saveToString",&Derived::saveToString,
             "Parses the current object to a string.")
        .def("loadFromString",&Derived::loadFromString,
             bp::arg("string"),
             "Parses from the input string the content of the current object.")
        .def("saveToXML",&Derived::saveToXML,
             bp::args("filename","tag_name"),"Saves *this inside a XML file.")
        .def("loadFromXML",&Derived::loadFromXML,
             bp::args("filename","tag_name"),"Loads *this from a XML file.")
        .def("saveToBinary",&Derived::saveToBinary,
             bp::arg("filename"),"Saves *this inside a binary file.")
        .def("loadFromBinary",&Derived::loadFromBinary,
             bp::arg("filename"),"Loads *this from a binary file.")
        ;
      }

    };
  }
}

#endif // ifndef __pinocchio_python_serialization_serializable_hpp__
