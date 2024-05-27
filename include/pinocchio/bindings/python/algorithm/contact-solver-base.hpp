//
// Copyright (c) 2022-2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/algorithm/contact-solver-base.hpp"

#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename Solver>
    struct ContactSolverBasePythonVisitor
    : public boost::python::def_visitor<ContactSolverBasePythonVisitor<Solver>>
    {
      typedef typename Solver::Scalar Scalar;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
        cl.def(
            "getIterationCount", &Solver::getIterationCount, bp::arg("self"),
            "Get the number of iterations achieved by the PGS algorithm.")
          .def(
            "setMaxIterations", &Solver::setMaxIterations, bp::args("self", "max_it"),
            "Set the maximum number of iterations.")
          .def(
            "getMaxIterations", &Solver::getMaxIterations, bp::arg("self"),
            "Get the maximum number of iterations allowed.")

          .def(
            "setAbsolutePrecision", &Solver::setAbsolutePrecision,
            bp::args("self", "absolute_precision"), "Set the absolute precision for the problem.")
          .def(
            "getAbsolutePrecision", &Solver::getAbsolutePrecision, bp::arg("self"),
            "Get the absolute precision requested.")

          .def(
            "setRelativePrecision", &Solver::setRelativePrecision,
            bp::args("self", "relative_precision"), "Set the relative precision for the problem.")
          .def(
            "getRelativePrecision", &Solver::getRelativePrecision, bp::arg("self"),
            "Get the relative precision requested.")

          .def(
            "getAbsoluteConvergenceResidual", &Solver::getAbsoluteConvergenceResidual,
            bp::arg("self"),
            "Returns the value of the absolute residual value corresponding to the contact "
            "complementary conditions.")
          .def(
            "getRelativeConvergenceResidual", &Solver::getRelativeConvergenceResidual,
            bp::arg("self"),
            "Returns the value of the relative residual value corresponding to the difference "
            "between two successive iterates (infinity norms).")

  #ifdef PINOCCHIO_WITH_HPP_FCL
          .def("getCPUTimes", &Solver::getCPUTimes, bp::arg("self"))
  #endif // PINOCCHIO_WITH_HPP_FCL
          ;
#endif
      }
    }; // struct ContactSolverBasePythonVisitor

  } // namespace python
} // namespace pinocchio
