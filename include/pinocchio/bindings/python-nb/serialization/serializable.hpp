// Copyright (c) 2026 INRIA

#pragma once

#include <sstream>

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/serialization.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<class Derived>
struct SerializableVisitor : public nb::def_visitor<SerializableVisitor<Derived>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
    cl.def("saveToText", &Derived::saveToText, "filename"_a, "Saves *this inside a text file.")
      .def("loadFromText", &Derived::loadFromText, "filename"_a, "Loads *this from a text file.")

      .def("saveToString", &Derived::saveToString, "Parses the current object to a string.")
      .def(
        "loadFromString", &Derived::loadFromString, "string"_a,
        "Parses from the input string the content of the current object.")

      .def(
        "saveToXML", &Derived::saveToXML, "filename"_a, "tag_name"_a,
        "Saves *this inside a XML file.")
      .def(
        "loadFromXML", &Derived::loadFromXML, "filename"_a, "tag_name"_a,
        "Loads *this from a XML file.")

      .def(
        "saveToBinary", (void (Derived::*)(const std::string &) const) & Derived::saveToBinary,
        "filename"_a, "Saves *this inside a binary file.")
      .def(
        "loadFromBinary", (void (Derived::*)(const std::string &))&Derived::loadFromBinary,
        "filename"_a, "Loads *this from a binary file.");
#else
    (void)cl;
#endif
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
