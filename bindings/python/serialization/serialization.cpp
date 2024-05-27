//
// Copyright (c) 2021-2023 INRIA
//

#include <boost/asio.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"
#include "pinocchio/bindings/python/serialization/serialization.hpp"

namespace pinocchio
{
  namespace python
  {

    static void buffer_copy(boost::asio::streambuf & dest, const boost::asio::streambuf & source)
    {
      std::size_t bytes_copied =
        boost::asio::buffer_copy(dest.prepare(source.size()), source.data());
      dest.commit(bytes_copied);
    }

    static boost::asio::streambuf &
    prepare_proxy(boost::asio::streambuf & self, const std::size_t n)
    {
      self.prepare(n);
      return self;
    }

    static PyObject * view(boost::asio::streambuf & self)
    {
      boost::asio::streambuf::const_buffers_type bufs = self.data();
      PyObject * py_buffer = PyMemoryView_FromMemory(
        const_cast<char *>(static_cast<const char *>(bufs.data())), self.size(), PyBUF_READ);
      return py_buffer;
    }

    static PyObject * tobytes(boost::asio::streambuf & self)
    {
      return PyBytes_FromObject(view(self));
    }

    void exposeSerialization()
    {
      namespace bp = boost::python;

      bp::scope current_scope = getOrCreatePythonNamespace("serialization");

      typedef boost::asio::streambuf StreamBuffer;
      if (!eigenpy::register_symbolic_link_to_registered_type<StreamBuffer>())
      {
        bp::class_<StreamBuffer, boost::noncopyable>(
          "StreamBuffer", "Stream buffer to save/load serialized objects in binary mode.",
          bp::init<>(bp::arg("self"), "Default constructor."))
          //      .def("capacity",&StreamBuffer::capacity,"Get the current capacity of the
          //      StreamBuffer.")
          .def("size", &StreamBuffer::size, "Get the size of the input sequence.")
          .def("max_size", &StreamBuffer::max_size, "Get the maximum size of the StreamBuffer.")
          .def("prepare", prepare_proxy, "Reserve data.", bp::return_self<>())
          .def(
            "view", view, "Returns the content of *this as a memory view.",
            bp::with_custodian_and_ward_postcall<0, 1>())
          .def("tobytes", tobytes, "Returns the content of *this as a byte sequence.");
      }

      typedef pinocchio::serialization::StaticBuffer StaticBuffer;
      if (!eigenpy::register_symbolic_link_to_registered_type<StaticBuffer>())
      {
        bp::class_<StaticBuffer>(
          "StaticBuffer",
          "Static buffer to save/load serialized objects in binary mode with pre-allocated memory.",
          bp::init<size_t>(
            bp::args("self", "size"), "Default constructor from a given size capacity."))
          .def("size", &StaticBuffer::size, bp::arg("self"), "Get the size of the input sequence.")
          .def(
            "reserve", &StaticBuffer::resize, bp::arg("new_size"),
            "Increase the capacity of the vector to a value that's greater or equal to new_size.");
      }

      bp::def(
        "buffer_copy", buffer_copy, bp::args("dest", "source"),
        "Copy bytes from a source buffer to a target buffer.");
    }

  } // namespace python
} // namespace pinocchio
