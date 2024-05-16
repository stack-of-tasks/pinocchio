//
// Copyright (c) 2022-2024 INRIA
//

#include "pinocchio/algorithm/pgs-solver.hpp"
#include "pinocchio/bindings/python/fwd.hpp"

#include "pinocchio/bindings/python/algorithm/contact-solver-base.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include <eigenpy/eigen-from-python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    typedef PGSContactSolverTpl<context::Scalar> Solver;

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    template<typename DelassusMatrixType>
    static bool solve_wrapper(
      Solver & solver,
      const DelassusMatrixType & G,
      const context::VectorXs & g,
      const context::CoulombFrictionConeVector & cones,
      Eigen::Ref<context::VectorXs> x,
      const context::Scalar over_relax = 1)
    {
      return solver.solve(G, g, cones, x, over_relax);
    }
#endif

    void exposePGSContactSolver()
    {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
      bp::class_<Solver>(
        "PGSContactSolver", "Projected Gauss Siedel solver for contact dynamics.",
        bp::init<int>(bp::args("self", "problem_dim"), "Default constructor."))
        .def(ContactSolverBasePythonVisitor<Solver>())
        .def(
          "solve", solve_wrapper<context::MatrixXs>,
          (bp::args("self", "G", "g", "cones", "x"), (bp::arg("over_relax") = context::Scalar(1))),
          "Solve the constrained conic problem composed of problem data (G,g,cones) and starting "
          "from the initial guess.")
        .def(
          "solve", solve_wrapper<context::SparseMatrix>,
          (bp::args("self", "G", "g", "cones", "x"), (bp::arg("over_relax") = context::Scalar(1))),
          "Solve the constrained conic problem composed of problem data (G,g,cones) and starting "
          "from the initial guess.");

#endif
    }

  } // namespace python
} // namespace pinocchio
