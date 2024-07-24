//
// Copyright (c) 2019-2022 INRIA
//

#ifndef __pinocchio_python_algorithm_proximal_hpp__
#define __pinocchio_python_algorithm_proximal_hpp__

#include "pinocchio/algorithm/proximal.hpp"

#include <sstream>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename ProximalSettings>
    struct ProximalSettingsPythonVisitor
    : public boost::python::def_visitor<ProximalSettingsPythonVisitor<ProximalSettings>>
    {
      typedef typename ProximalSettings::Scalar Scalar;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::init<>("Default constructor.", bp::arg("self")))
          .def(bp::init<const Scalar, const Scalar, int>(
            (bp::arg("self"), bp::arg("accuracy"), bp::arg("mu"), bp::arg("max_iter")),
            "Structure containing all the settings parameters for the proximal algorithms."))
          .def(bp::init<const Scalar, const Scalar, const Scalar, int>(
            (bp::arg("self"), bp::arg("absolute_accuracy"), bp::arg("relative_accuracy"),
             bp::arg("mu"), bp::arg("max_iter")),
            "Structure containing all the settings parameters for the proximal algorithms."))

          .add_property(
            "absolute_accuracy", &ProximalSettings::absolute_accuracy,
            "Absolute proximal accuracy.")
          .add_property(
            "relative_accuracy", &ProximalSettings::relative_accuracy,
            "Relative proximal accuracy between two iterates.")
          .add_property(
            "mu", &ProximalSettings::mu, "Regularization parameter of the Proximal algorithms.")
          .add_property("max_iter", &ProximalSettings::max_iter, "Maximal number of iterations.")

          .add_property(
            "absolute_residual", &ProximalSettings::absolute_residual, "Absolute residual.")
          .add_property(
            "relative_residual", &ProximalSettings::relative_residual,
            "Relatice residual between two iterates.")

          .add_property(
            "iter", &ProximalSettings::iter,
            "Final number of iteration of the algorithm when it has converged or "
            "reached the maximal number of allowed iterations.")
          .def("__repr__", &repr);
      }

      static void expose()
      {
        bp::class_<ProximalSettings>(
          "ProximalSettings",
          "Structure containing all the settings parameters for proximal algorithms.", bp::no_init)
          .def(ProximalSettingsPythonVisitor<ProximalSettings>());
      }

    private:
      static std::string repr(const ProximalSettings & self)
      {
        std::stringstream ss_repr;

        ss_repr << "ProximalSettings(";
        ss_repr << self.absolute_accuracy << ", ";
        ss_repr << self.relative_accuracy << ", ";
        ss_repr << self.mu << ", ";
        ss_repr << self.max_iter;
        ss_repr << ")";

        return ss_repr.str();
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_proximal_hpp__
