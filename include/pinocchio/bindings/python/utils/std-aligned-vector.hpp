//
// Copyright (c) 2016-2024 CNRS INRIA
//

#ifndef __pinocchio_python_utils_std_aligned_vector_hpp__
#define __pinocchio_python_utils_std_aligned_vector_hpp__

#include <boost/python.hpp>
#include <string>

#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/bindings/python/utils/pickle-vector.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace pinocchio
{
  namespace python
  {

    ///
    /// \brief Expose an container::aligned_vector from a type given as template argument.
    ///
    /// \tparam T Type to expose as container::aligned_vector<T>.
    /// \tparam EnableFromPythonListConverter Enables the conversion from a Python list to a
    /// container::aligned_vector<T>.
    ///
    /// \sa StdAlignedVectorPythonVisitor
    ///
    template<class T, bool NoProxy = false, bool EnableFromPythonListConverter = true>
    struct StdAlignedVectorPythonVisitor
    : public ::boost::python::vector_indexing_suite<
        typename container::aligned_vector<T>,
        NoProxy,
        eigenpy::internal::
          contains_vector_derived_policies<typename container::aligned_vector<T>, NoProxy>>
    , public eigenpy::StdContainerFromPythonList<container::aligned_vector<T>>
    {
      typedef container::aligned_vector<T> vector_type;
      typedef eigenpy::StdContainerFromPythonList<vector_type, NoProxy> FromPythonListConverter;
      typedef T value_type;

      static void expose(const std::string & class_name, const std::string & doc_string = "")
      {
        expose(class_name, doc_string, eigenpy::EmptyPythonVisitor());
      }

      template<typename VisitorDerived>
      static void expose(
        const std::string & class_name, const boost::python::def_visitor<VisitorDerived> & visitor)
      {
        expose(class_name, "", visitor);
      }

      template<typename VisitorDerived>
      static void expose(
        const std::string & class_name,
        const std::string & doc_string,
        const boost::python::def_visitor<VisitorDerived> & visitor)
      {
        namespace bp = boost::python;
        if (!eigenpy::register_symbolic_link_to_registered_type<vector_type>())
        {
          bp::class_<vector_type> cl(class_name.c_str(), doc_string.c_str());
          cl.def(StdAlignedVectorPythonVisitor())

            .def(bp::init<size_t, const value_type &>(
              bp::args("self", "size", "value"),
              "Constructor from a given size and a given value."))
            .def(bp::init<const vector_type &>(bp::args("self", "other"), "Copy constructor"))

            .def(
              "tolist", &FromPythonListConverter::tolist,
              (bp::arg("self"), bp::arg("deep_copy") = false),
              "Returns the aligned_vector as a Python list.")
            .def(visitor)
#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
            .def_pickle(PickleVector<vector_type>())
#endif
            .def(eigenpy::CopyableVisitor<vector_type>());

          // Register conversion
          if (EnableFromPythonListConverter)
            FromPythonListConverter::register_converter();
        }
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_aligned_vector_hpp__
