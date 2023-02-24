//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/pgs-solver.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
namespace python
{
  namespace bp = boost::python;

  typedef PGSContactSolverTpl<context::Scalar> Solver;

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
  static bool solve_wrapper(Solver & solver, const context::MatrixXs & G,
                            const context::VectorXs & g, const context::CoulombFrictionConeVector & cones,
                            Eigen::Ref<context::VectorXs> x,
                            const context::Scalar over_relax = 1)
  {
    return solver.solve(G,g,cones,x,over_relax);
  }
#endif

  void exposePGSContactSolver()
  {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::class_<Solver>("PGSSolver",
                       "Projected Gauss Siedel solver.",
                       bp::init<int>(bp::args("self","problem_dim"),"Default constructor."))

    .def("solve",solve_wrapper,(bp::args("self","G","g","cones","x"),(bp::arg("over_relax") = context::Scalar(1))),
         "Solve the constrained conic problem composed of problem data (G,g,cones) and starting from the initial guess.")

    .def("getIterationCount",&Solver::getIterationCount,bp::arg("self"),
         "Get the number of iterations achieved by the PGS algorithm.")
    .def("setMaxIterations",&Solver::setMaxIterations,bp::args("self","max_it"),
         "Set the maximum number of iterations.")
    .def("getMaxIterations",&Solver::getMaxIterations,bp::arg("self"),
         "Get the maximum number of iterations allowed.")

    .def("setAbsolutePrecision",&Solver::setAbsolutePrecision,bp::args("self","absolute_precision"),
         "Set the absolute precision for the problem.")
    .def("getAbsolutePrecision",&Solver::getAbsolutePrecision,bp::arg("self"),
         "Get the absolute precision requested.")

    .def("setRelativePrecision",&Solver::setRelativePrecision,bp::args("self","relative_precision"),
         "Set the relative precision for the problem.")
    .def("getRelativePrecision",&Solver::getRelativePrecision,bp::arg("self"),
         "Get the relative precision requested.")

    .def("getAbsoluteConvergenceResidual",&Solver::getAbsoluteConvergenceResidual,bp::arg("self"),
         "Returns the value of the absolute residual value corresponding to the contact complementary conditions.")
    .def("getRelativeConvergenceResidual",&Solver::getRelativeConvergenceResidual,bp::arg("self"),
         "Returns the value of the relative residual value corresponding to the difference between two successive iterates (infinity norms).")
    ;
#endif
  }


} // namespace python
} // namespace pinocchio
