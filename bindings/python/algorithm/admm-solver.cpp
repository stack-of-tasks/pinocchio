//
// Copyright (c) 2024 INRIA
//

#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>

#include "pinocchio/bindings/python/fwd.hpp"

#include "pinocchio/algorithm/admm-solver.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/delassus-operator-plain.hpp"
#include "pinocchio/algorithm/delassus-operator-sparse.hpp"

#include "pinocchio/bindings/python/algorithm/contact-solver-base.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/macros.hpp"


namespace pinocchio
{
namespace python
{
  namespace bp = boost::python;

  typedef ADMMContactSolverTpl<context::Scalar> Solver;
  typedef Solver::PowerIterationAlgo PowerIterationAlgo;
  typedef Solver::SolverStats SolverStats;
  typedef context::Scalar Scalar;
  typedef context::VectorXs VectorXs;
  typedef const Eigen::Ref<const VectorXs> ConstRefVectorXs;
  typedef ContactCholeskyDecompositionTpl<context::Scalar,context::Options> ContactCholeskyDecomposition;

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE

  template<typename DelassusDerived>
  static bool solve_wrapper(Solver & solver,
                            DelassusDerived & delassus,
                            const context::VectorXs & g,
                            const context::CoulombFrictionConeVector & cones,
                            const context::VectorXs & R,
                            const boost::optional<ConstRefVectorXs> primal_solution = boost::none,
                            const boost::optional<ConstRefVectorXs> dual_solution = boost::none,
                            bool compute_largest_eigen_values = true,
                            bool stat_record = false)
  {
    return solver.solve(delassus,g,cones,R,primal_solution,dual_solution,compute_largest_eigen_values);
  }

  template<typename DelassusDerived>
  static bool solve_wrapper2(Solver & solver,
                             DelassusDerived & delassus,
                             const context::VectorXs & g,
                             const context::CoulombFrictionConeVector & cones,
                             Eigen::Ref<context::VectorXs> x)
  {
    return solver.solve(delassus,g,cones,x);
  }
#endif

  void exposeADMMContactSolver()
  {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::class_<Solver>("ADMMContactSolver",
                       "Alternating Direction Method of Multi-pliers solver for contact dynamics.",
                       bp::init<int, Scalar, Scalar, Scalar, Scalar, Scalar, int>((bp::arg("self"),
                                                                                   bp::arg("problem_dim"),
                                                                                   bp::arg("mu_prox") = Scalar(1e-6),
                                                                                   bp::arg("tau") = Scalar(0.5),
                                                                                   bp::arg("rho_power") = Scalar(0.2),
                                                                                   bp::arg("rho_power_factor") = Scalar(0.05),
                                                                                   bp::arg("ratio_primal_dual") = Scalar(10),
                                                                                   bp::arg("max_it_largest_eigen_value_solver") = 20),
                                                                     "Default constructor."))
    .def(ContactSolverBasePythonVisitor<Solver>())

    .def("solve",solve_wrapper<ContactCholeskyDecomposition::DelassusCholeskyExpression>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true,
          bp::arg("stat_record") = false),
         "Solve the constrained conic problem, starting from the optional initial guess.")
    .def("solve",solve_wrapper<context::DelassusOperatorDense>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true,
          bp::arg("stat_record") = false),
         "Solve the constrained conic problem, starting from the optional initial guess.")
    .def("solve",solve_wrapper<context::DelassusOperatorSparse>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true,
          bp::arg("stat_record") = false),
         "Solve the constrained conic problem, starting from the optional initial guess.")

    .def("setRho",&Solver::setRho,bp::args("self","rho"),
         "Set the ADMM penalty value.")
    .def("getRho",&Solver::getRho,bp::arg("self"),
         "Get the ADMM penalty value.")

    .def("setRhoPower",&Solver::setRhoPower,bp::args("self","rho_power"),
         "Set the power associated to the problem conditionning.")
    .def("getRhoPower",&Solver::getRhoPower,bp::arg("self"),
         "Get the power associated to the problem conditionning.")

    .def("setRhoPowerFactor",&Solver::setRhoPowerFactor,bp::args("self","rho_power_factor"),
         "Set the power factor associated to the problem conditionning.")
    .def("getRhoPowerFactor",&Solver::getRhoPowerFactor,bp::arg("self"),
         "Get the power factor associated to the problem conditionning.")

    .def("setTau",&Solver::setTau,bp::args("self","tau"),
         "Set the tau linear scaling factor.")
    .def("getTau",&Solver::getTau,bp::arg("self"),
         "Get the tau linear scaling factor.")

    .def("setProximalValue",&Solver::setProximalValue,bp::args("self","mu"),
         "Set the proximal value.")
    .def("getProximalValue",&Solver::getProximalValue,bp::arg("self"),
         "Get the proximal value.")

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

    .def("computeRho",&Solver::computeRho,bp::args("L","m","rho_power"),
         "Compute the penalty ADMM value from the current largest and lowest Eigen values and the scaling spectral factor.")
    .staticmethod("computeRho")
    .def("computeRhoPower",&Solver::computeRhoPower,bp::args("L","m","rho"),
         "Compute the  scaling spectral factor of the ADMM penalty term from the current largest and lowest Eigen values and the ADMM penalty term.")
    .staticmethod("computeRhoPower")

    .def("getPowerIterationAlgo",
         &Solver::getPowerIterationAlgo,
         bp::arg("self"),
         bp::return_internal_reference<>())

    .def("getStats",
         &Solver::getStats,
         bp::arg("self"),
         bp::return_internal_reference<>())
    ;

    {
      bp::class_<PowerIterationAlgo>("PowerIterationAlgo","",
                                     bp::init<Eigen::DenseIndex, int, Scalar>((bp::arg("self"),
                                                                               bp::arg("max_it") = 10,
                                                                               bp::arg("rel_tol") = 1e-8),
                                                                              "Default constructor"))

      .PINOCCHIO_ADD_PROPERTY(PowerIterationAlgo,principal_eigen_vector,"Principal eigen vector.")
      .PINOCCHIO_ADD_PROPERTY(PowerIterationAlgo,max_it,"Maximum number of iterations.")
      .PINOCCHIO_ADD_PROPERTY(PowerIterationAlgo,rel_tol,"Relative tolerance.")
      .PINOCCHIO_ADD_PROPERTY_READONLY(PowerIterationAlgo,largest_eigen_value,"Largest eigen value.")
      .PINOCCHIO_ADD_PROPERTY_READONLY(PowerIterationAlgo,it,"Number of iterations performed by the algorithm.")
      .PINOCCHIO_ADD_PROPERTY_READONLY(PowerIterationAlgo,convergence_criteria,"Convergence criteria to quit the algorithm.")
      ;
    }

    {
      bp::class_<SolverStats>("SolverStats","",
                              bp::init<int>((bp::arg("self"),
                                             bp::arg("max_it")),
                                            "Default constructor"))
      .def("reset",&SolverStats::reset,bp::arg("self"),"Reset the stasts.")
      .def("size",&SolverStats::size,bp::arg("self"),"Size of the vectors stored in the structure.")

      .PINOCCHIO_ADD_PROPERTY_READONLY(SolverStats,primal_feasibility,"")
      .PINOCCHIO_ADD_PROPERTY_READONLY(SolverStats,dual_feasibility,"")
      .PINOCCHIO_ADD_PROPERTY_READONLY(SolverStats,complementarity,"")
      .PINOCCHIO_ADD_PROPERTY_READONLY(SolverStats,it,"Number of iterations performed by the algorithm.")
      .PINOCCHIO_ADD_PROPERTY_READONLY(SolverStats,cholesky_update_count,"Number of Cholesky updates performed by the algorithm.")
      ;
    }
#endif
  }


} // namespace python
} // namespace pinocchio
