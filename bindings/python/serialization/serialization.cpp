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
    
    void exposeSerialization()
    {
      namespace bp = boost::python;
      
      bp::scope current_scope = getOrCreatePythonNamespace("serialization");
      
      bp::class_<boost::asio::streambuf,boost::noncopyable>("StreamBuffer",
                                                            "Stream buffer to save/load serialized objects in binary mode.",
                                                            bp::init<>(bp::arg("self"),"Default constructor."));
    }
    
  } // namespace python
} // namespace pinocchio
