//
// Copyright (c) 2021 INRIA
//

#include <boost/asio/streambuf.hpp>

#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

namespace pinocchio
{
  namespace python
  {
    
    static void buffer_copy(boost::asio::streambuf & dest,
                            const boost::asio::streambuf & source)
    {
      boost::asio::buffer_copy(dest.prepare(source.size()),source.data());
    }
    
    void exposeSerialization()
    {
      namespace bp = boost::python;
      
      bp::scope current_scope = getOrCreatePythonNamespace("serialization");
      
      bp::class_<boost::asio::streambuf,boost::noncopyable>("StreamBuffer",
                                                            "Stream buffer to save/load serialized objects in binary mode.",
                                                            bp::init<>(bp::arg("self"),"Default constructor."));
      
      bp::def("buffer_copy",buffer_copy,
              bp::args("dest","source"),
              "Copy bytes from a source buffer to a target buffer.");
    }
    
  } // namespace python
} // namespace pinocchio
