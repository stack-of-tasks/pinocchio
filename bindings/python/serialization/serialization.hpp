//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_serialization_serialization_hpp__
#define __pinocchio_python_serialization_serialization_hpp__

#include "pinocchio/serialization/archive.hpp"

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeSerialization();
  
    template<typename T>
    void serialize()
    {
      namespace bp = boost::python;
      
      bp::scope current_scope = getOrCreatePythonNamespace("serialization");
      
      bp::def("loadFromBinary",(void (*)(T &, boost::asio::streambuf &))pinocchio::serialization::loadFromBinary<T>,
              bp::args("object","stream_buffer"),
              "Load an object from a binary buffer.");
      
      bp::def("saveToBinary",(void (*)(const T &, boost::asio::streambuf &))pinocchio::serialization::saveToBinary<T>,
              bp::args("object","stream_buffer"),
              "Save an object to a binary buffer.");
      
      bp::def("loadFromBinary",(void (*)(T &, serialization::StaticBuffer &))pinocchio::serialization::loadFromBinary<T>,
              bp::args("object","static_buffer"),
              "Load an object from a static binary buffer.");
      
      bp::def("saveToBinary",(void (*)(const T &, serialization::StaticBuffer &))pinocchio::serialization::saveToBinary<T>,
              bp::args("object","static_buffer"),
              "Save an object to a static binary buffer.");
    }
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_serialization_serialization_hpp__
