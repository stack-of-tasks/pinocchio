// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/comparable.hpp"
#include "pinocchio/bindings/python-nb/utils/printable.hpp"
#include "pinocchio/bindings/python-nb/constraints/constraint-crtp-base.hpp"

#include <boost/mpl/for_each.hpp>
#include <boost/algorithm/string/replace.hpp>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

// constraints/cones.cpp
void exposeCones(nb::module_ m);
// constraints/constraint-solvers.cpp
void exposeConstraintSolvers(nb::module_ m);

static void exposeBaumgarteParameters(nb::module_ m)
{
  nb::class_<BaumgarteCorrectorParameters>(
    m, "BaumgarteCorrectorParameters", "Parameters of the Baumgarte corrector.")
    .def(nb::init<>())
    .def(nb::init<Scalar, Scalar>(), "Kp"_a, "Kd"_a)
    .def_rw("Kp", &BaumgarteCorrectorParameters::Kp, "Proportional corrector gain value.")
    .def_rw("Kd", &BaumgarteCorrectorParameters::Kd, "Damping corrector gain value.")
    .def(ComparableVisitor<BaumgarteCorrectorParameters>());
}

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
      .def(PrintableVisitor<Derived>());
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
      .def(PrintableVisitor<Derived>());
  }
  void operator()(BlankConstraintData)
  {
  }
};

void exposeConstraints(nb::module_ m)
{
  exposeBaumgarteParameters(m);

  // Expose constraint collection

  // loop over model derived types
  using model_types = ConstraintModelVariant::types;
  boost::mpl::for_each<model_types>(model_callable{m});

  // loop over data derived types
  using data_types = ConstraintDataVariant::types;
  boost::mpl::for_each<data_types>(data_callable{m});
}
PINOCCHIO_PYTHON_NAMESPACE_END
