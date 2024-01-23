//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"

#include "pinocchio/algorithm/admm-solver.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/algorithm/contact-solver-base.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
namespace python
{
  namespace bp = boost::python;

  typedef ADMMContactSolverTpl<context::Scalar> Solver;
  typedef cholesky::ContactCholeskyDecompositionTpl<context::Scalar,context::Options> ContactCholeskyDecomposition;

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
  static bool solve_wrapper(Solver & solver, 
                            ContactCholeskyDecomposition::DelassusCholeskyExpression & delassus_expression,
                            const context::VectorXs & g,
                            const context::CoulombFrictionConeVector & cones,
                            Eigen::Ref<context::VectorXs> x,
                            const context::Scalar mu_prox,
                            const context::VectorXs & R,
                            const context::Scalar tau = context::Scalar(0.99))
  {
    return solver.solve(delassus_expression,g,cones,x,mu_prox,R,tau);
  }
#endif

  void exposeADMMContactSolver()
  {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::class_<Solver>("ADMMContactSolver",
                       "Alternating Direction Method of Multi-pliers solver for contact dynamics.",
                       bp::init<int>(bp::args("self","problem_dim"),"Default constructor."))
    .def(ContactSolverBasePythonVisitor<Solver>())
    .def("solve",solve_wrapper,(bp::args("self","delassus_expression","g","cones","x","mu_prox","R"),(bp::arg("tau") = context::Scalar(0.99))),
         "Solve the constrained conic problem, starting from the initial guess.")

    .def("setRhoPower",&Solver::setRhoPower,bp::args("self","rho_power"),
         "Set the power associated to the problem conditionning.")
    .def("getRhoPower",&Solver::getRhoPower,bp::arg("self"),
         "Get the power associated to the problem conditionning.")

    .def("setRatioPrimalDual",&Solver::setRatioPrimalDual,bp::args("self","ratio_primal_dual"),
         "Set the primal/dual ratio.")
    .def("getRatioPrimalDual",&Solver::getRatioPrimalDual,bp::arg("self"),
         "Get the primal/dual ratio.")
    
    .def("getPrimalSolution",&Solver::getPrimalSolution,bp::arg("self"),
         "Returns the primal solution of the problem.",bp::return_internal_reference<>())

    .def("getDualSolution",&Solver::getDualSolution,bp::arg("self"),
         "Returns the dual solution of the problem.",bp::return_internal_reference<>())

    .def("getCholeskyUpdateCount",&Solver::getCholeskyUpdateCount,bp::arg("self"),
         "Returns the number of updates of the Cholesky factorization due to rho updates.")
    ;
#endif
  }


} // namespace python
} // namespace pinocchio
