// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/utils/printable.hpp"
#include "pinocchio/bindings/python-nb/constraints/constraint-crtp-base.hpp"

#include <boost/mpl/for_each.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<typename T>
std::string sanitizedClassname(T *)
{
  std::string className = boost::replace_all_copy(T::classname(), "<", "_");
  boost::replace_all(className, ">", "");
  return className;
}

std::string sanitizedClassname(BlankConstraintModel *)
{
  return "BlankConstraintModel";
}

std::string sanitizedClassname(BlankConstraintData *)
{
  return "BlankConstraintData";
}

struct model_callable
{
  nb::module_ m;
  template<typename Derived>
  void operator()(Derived)
  {
    auto className = sanitizedClassname((Derived *)0);
    nb::class_<Derived>(m, className.c_str())
      .def(ConstraintModelBaseVisitor<Derived>())
      .def(PrintableVisitor<Derived>());
    auto vecName = "StdVec_" + className;
    nb::bind_vector<std::vector<Derived>, nb::rv_policy::reference_internal>(m, vecName.c_str());
  }
  void operator()(BlankConstraintModel)
  {
  }
};

struct data_callable
{
  nb::module_ m;
  template<typename Derived>
  void operator()(Derived)
  {
    auto className = sanitizedClassname((Derived *)0);
    nb::class_<Derived>(m, className.c_str())
      .def(ConstraintDataBaseVisitor<Derived>())
      .def(PrintableVisitor<Derived>());
    auto vecName = "StdVec_" + className;
    nb::bind_vector<std::vector<Derived>, nb::rv_policy::reference_internal>(m, vecName.c_str());
  }
  void operator()(BlankConstraintData)
  {
  }
};

void exposeConstraintCollection(nb::module_ m)
{
  // loop over model derived types
  using model_types = ConstraintModelVariant::types;
  boost::mpl::for_each<model_types>(model_callable{m});

  // loop over data derived types
  using data_types = ConstraintDataVariant::types;
  boost::mpl::for_each<data_types>(data_callable{m});
}

PINOCCHIO_PYTHON_NAMESPACE_END
