// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/multibody/joint-generic.hpp"
#include "pinocchio/bindings/python-nb/multibody/joint-crtp-base.hpp"

#include <boost/mpl/for_each.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
template<typename T>
std::string sanitizedClassname()
{
  std::string className = boost::replace_all_copy(T::classname(), "<", "_");
  boost::replace_all(className, ">", "");
  return className;
}

struct model_callable
{
  nb::module_ m;
  template<typename Derived>
  void operator()(Derived)
  {
    auto className = sanitizedClassname<Derived>();
    nb::class_<Derived>(m, className.c_str())
      .def(JointModelBaseVisitor<Derived>())
      .def(PrintableVisitor<Derived>());
  }
};
struct data_callable
{
  nb::module_ m;
  template<typename Derived>
  void operator()(Derived)
  {
    auto className = sanitizedClassname<Derived>();
    nb::class_<Derived>(m, className.c_str())
      .def(JointDataBaseVisitor<Derived>())
      .def(PrintableVisitor<Derived>());
  }
};

// Expose all joints in the default collection
void exposeJointCollection(nb::module_ m)
{
  // loop over model derived types
  using model_types = JointModelVariant::types;
  boost::mpl::for_each<model_types>(model_callable{m});

  // loop over data derived types
  using data_types = JointDataVariant::types;
  boost::mpl::for_each<data_types>(data_callable{m});
}

PINOCCHIO_PYTHON_NAMESPACE_END
