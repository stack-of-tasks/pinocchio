// Copyright (c) 2026 INRIA

// Port of the following files:
// - bindings/python/algorithm/solvers/expose-pgs-solver.cpp
// - bindings/python/algorithm/solvers/expose-admm-solver.cpp

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/algorithm/constraint-solver-base.hpp"

#include "pinocchio/algorithm/solvers/pgs-solver.hpp"
#include "pinocchio/algorithm/solvers/admm-solver.hpp"
#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/delassus-operator.hpp"

#include <nanobind/stl/optional.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using PGSSolver = PGSConstraintSolverTpl<Scalar, Options>;
using PGSSolverSettings = typename PGSSolver::PGSSolverSettings;
using PGSSolverResult = typename PGSSolver::PGSSolverResult;
using PGSSolverStats = typename PGSSolver::PGSSolverStats;

using ADMMSolver = ADMMConstraintSolverTpl<Scalar, Options>;
using ADMMSolverSettings = typename ADMMSolver::ADMMSolverSettings;
using ADMMSolverResult = typename ADMMSolver::ADMMSolverResult;
using ADMMSolverStats = typename ADMMSolver::ADMMSolverStats;

using DelassusOperatorCholesky =
  typename ConstraintCholeskyDecomposition::DelassusOperatorCholeskyExpression;
using DelassusOperatorDense = DelassusOperatorDenseTpl<Scalar, Options>;
using DelassusOperatorSparse = DelassusOperatorSparseTpl<Scalar, Options>;

// PGS Solver
static void exposePGS(nb::module_ m)
{
  // --- PGSSolverSettings ---
  nb::class_<PGSSolverSettings>(m, "PGSSolverSettings", "Settings for the PGS constraint solver.")
    .def(nb::init<>(), "Default constructor with default settings.")
    .def(ConstraintSolverSettingsBaseVisitor<PGSSolverSettings>())
    // PGS-specific settings
    .def_rw(
      "over_relaxation", &PGSSolverSettings::over_relaxation,
      "Over-relaxation parameter (should be in ]0,2[, default 1).");

  // --- PGSSolverResult ---
  nb::class_<PGSSolverResult>(m, "PGSSolverResult", "Solution of the PGS constraint solver.")
    .def(nb::init<>(), "Default constructor.")
    .def(ConstraintSolverResultBaseVisitor<PGSSolverResult>())
    // PGS-specific result fields
    .def_ro("problem_size", &PGSSolverResult::problem_size, "Problem size.")
    .def("resize", &PGSSolverResult::resize, "problem_size"_a, "Resize solution vectors.")
    .def(
      "retrievePrimalSolution",
      [](const PGSSolverResult & self, Eigen::Ref<VectorXs> primal_solution) {
        self.retrievePrimalSolution(primal_solution);
      },
      "primal_solution"_a, "Retrieve the primal solution (copy x into primal_solution).")
    .def(
      "retrieveDualSolution",
      [](const PGSSolverResult & self, Eigen::Ref<VectorXs> dual_solution) {
        self.retrieveDualSolution(dual_solution);
      },
      "dual_solution"_a, "Retrieve the dual solution (copy y into dual_solution).");

  // --- PGSSolverStats ---
  nb::class_<PGSSolverStats>(
    m, "PGSSolverStats", "Per-iteration statistics of the PGS constraint solver.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<std::size_t>(), "max_iterations"_a, "Constructor with maximum iterations.")
    .def(ConstraintSolverStatsBaseVisitor<PGSSolverStats>());

  // --- PGSConstraintSolver ---
  nb::class_<PGSSolver>(
    m, "PGSConstraintSolver", "Projected Gauss-Seidel (PGS) solver for contact dynamics.")
    .def(nb::init<std::size_t>(), "problem_size"_a, "Constructor with problem dimension.")
    .def(ConstraintSolverBaseVisitor<PGSSolver>())
    // PGS-specific
    .def_prop_ro(
      "stats", [](const PGSSolver & self) -> const PGSSolverStats & { return self.stats; },
      nb::rv_policy::reference_internal, "Access the statistics of the solver.")
    .def(
      "isValid", &PGSSolver::isValid,
      "Check if the solver is in a valid state (has solved a constraint problem).")
    // solve() with ConstraintCholeskyDecomposition::DelassusOperatorCholeskyExpression
    .def(
      "solve",
      [](
        PGSSolver & self, DelassusOperatorCholesky & delassus, ConstVectorRef g,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas, const PGSSolverSettings & settings,
        PGSSolverResult & result) -> bool {
        return self.solve(delassus, g, constraint_models, constraint_datas, settings, result);
      },
      "delassus"_a, "g"_a, "constraint_models"_a, "constraint_datas"_a, "settings"_a, "result"_a,
      "Solve the constrained conic problem with given settings and result.")
    // solve() with DelassusOperatorDense
    .def(
      "solve",
      [](
        PGSSolver & self, DelassusOperatorDense & delassus, ConstVectorRef g,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas, const PGSSolverSettings & settings,
        PGSSolverResult & result) -> bool {
        return self.solve(delassus, g, constraint_models, constraint_datas, settings, result);
      },
      "delassus"_a, "g"_a, "constraint_models"_a, "constraint_datas"_a, "settings"_a, "result"_a,
      "Solve the constrained conic problem with given settings and result.");
}

// ADMM Solver
static void exposeADMM(nb::module_ m)
{
  // --- ADMMUpdateRule enum ---
  nb::enum_<ADMMUpdateRule>(m, "ADMMUpdateRule")
    .value("SPECTRAL", ADMMUpdateRule::SPECTRAL)
    .value("OSQP", ADMMUpdateRule::OSQP)
    .value("LINEAR", ADMMUpdateRule::LINEAR)
    .value("CONSTANT", ADMMUpdateRule::CONSTANT);

  // --- ADMMProximalRule enum ---
  nb::enum_<ADMMProximalRule>(m, "ADMMProximalRule")
    .value("MANUAL", ADMMProximalRule::MANUAL)
    .value("AUTOMATIC", ADMMProximalRule::AUTOMATIC);

  // --- ADMMSolverSettings ---
  nb::class_<ADMMSolverSettings>(
    m, "ADMMSolverSettings", "Settings for the ADMM constraint solver.")
    .def(nb::init<>(), "Default constructor with default settings.")
    .def(ConstraintSolverSettingsBaseVisitor<ADMMSolverSettings>())
    // ADMM-specific settings
    .def_prop_rw(
      "rho_init",
      [](const ADMMSolverSettings & self) -> std::optional<Scalar> { return self.rho_init; },
      [](ADMMSolverSettings & self, std::optional<Scalar> v) { self.rho_init = v; },
      "Initial value of rho parameter (optional). If None, will be estimated from Delassus.")
    .def_rw(
      "warmstart_rho_with_previous_result", &ADMMSolverSettings::warmstart_rho_with_previous_result,
      "Whether to warmstart rho with previous result.")
    .def_rw("admm_update_rule", &ADMMSolverSettings::admm_update_rule, "ADMM update rule.")
    .def_rw("admm_proximal_rule", &ADMMSolverSettings::admm_proximal_rule, "ADMM proximal rule.")
    .def_rw("mu_prox", &ADMMSolverSettings::mu_prox, "Proximal penalty parameter.")
    .def_rw("tau_prox", &ADMMSolverSettings::tau_prox, "Proximal scaling factor.")
    .def_rw("tau", &ADMMSolverSettings::tau, "ADMM penalty scaling factor.")
    .def_rw(
      "ratio_primal_dual", &ADMMSolverSettings::ratio_primal_dual,
      "Primal/dual ratio threshold for rho update.")
    .def_rw("dual_momentum", &ADMMSolverSettings::dual_momentum, "Dual variable momentum.")
    .def_rw(
      "rho_update_ratio", &ADMMSolverSettings::rho_update_ratio, "Ratio threshold for rho update.")
    .def_rw(
      "rho_min_update_frequency", &ADMMSolverSettings::rho_min_update_frequency,
      "Minimum frequency for rho updates.")
    .def_rw("rho_momentum", &ADMMSolverSettings::rho_momentum, "Momentum on rho updates.")
    .def_rw("rho_min", &ADMMSolverSettings::rho_min, "Minimum value for rho parameter.")
    .def_rw("rho_max", &ADMMSolverSettings::rho_max, "Maximum value for rho parameter.")
    .def_rw(
      "spectral_rho_power_init", &ADMMSolverSettings::spectral_rho_power_init,
      "Initial rho power for SPECTRAL rule.")
    .def_rw(
      "spectral_rho_power_factor", &ADMMSolverSettings::spectral_rho_power_factor,
      "Rho power factor for SPECTRAL rule.")
    .def_rw(
      "linear_update_rule_factor", &ADMMSolverSettings::linear_update_rule_factor,
      "Factor for LINEAR update rule.")
    .def_rw(
      "lanczos_size", &ADMMSolverSettings::lanczos_size,
      "Size of Lanczos decomposition for eigenvalue estimation.")
    .def_rw(
      "max_delassus_decomposition_updates", &ADMMSolverSettings::max_delassus_decomposition_updates,
      "Maximum number of Delassus decomposition updates.")
    .def_rw(
      "anderson_capacity", &ADMMSolverSettings::anderson_capacity,
      "Anderson acceleration capacity.");

  // --- ADMMSolverResult ---
  nb::class_<ADMMSolverResult>(m, "ADMMSolverResult", "Solution of the ADMM constraint solver.")
    .def(nb::init<>(), "Default constructor.")
    .def(ConstraintSolverResultBaseVisitor<ADMMSolverResult>())
    // ADMM-specific result fields
    .def_ro("problem_size", &ADMMSolverResult::problem_size, "Problem size.")
    .def_ro(
      "delassus_decomposition_update_count", &ADMMSolverResult::delassus_decomposition_update_count,
      "Number of Delassus decomposition updates.")
    .def_ro("rho", &ADMMSolverResult::rho, "Final rho value.")
    .def_ro(
      "spectral_rho_power", &ADMMSolverResult::spectral_rho_power, "Final spectral rho power.")
    .def_ro("mu_prox", &ADMMSolverResult::mu_prox, "Final proximal parameter.")
    .def("resize", &ADMMSolverResult::resize, "problem_size"_a, "Resize solution vectors.")
    .def(
      "retrievePrimalSolution",
      [](const ADMMSolverResult & self, Eigen::Ref<VectorXs> primal_solution) {
        self.retrievePrimalSolution(primal_solution);
      },
      "primal_solution"_a, "Retrieve the primal solution (copy y into primal_solution).")
    .def(
      "retrieveDualSolution",
      [](const ADMMSolverResult & self, Eigen::Ref<VectorXs> dual_solution) {
        self.retrieveDualSolution(dual_solution);
      },
      "dual_solution"_a, "Retrieve the dual solution (copy z into dual_solution).")
    .def(
      "retrieveDesaxceTerm",
      [](const ADMMSolverResult & self, Eigen::Ref<VectorXs> desaxce_term) {
        self.retrieveDesaxceTerm(desaxce_term);
      },
      "desaxce_term"_a, "Retrieve the DeSaxce correction term (copy desaxce into desaxce_term).");

  // --- ADMMSolverStats ---
  nb::class_<ADMMSolverStats>(
    m, "ADMMSolverStats", "Per-iteration statistics of the ADMM constraint solver.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<std::size_t>(), "max_iterations"_a, "Constructor with maximum iterations.")
    .def(ConstraintSolverStatsBaseVisitor<ADMMSolverStats>())
    // ADMM-specific stats
    .def_ro("rho", &ADMMSolverStats::rho, "History of rho values.")
    .def_ro("mu_prox", &ADMMSolverStats::mu_prox, "History of mu_prox values.")
    .def_ro(
      "anderson_size", &ADMMSolverStats::anderson_size, "History of Anderson acceleration size.")
    .def_ro(
      "linear_system_residual", &ADMMSolverStats::linear_system_residual,
      "History of linear system residuals.")
    .def_ro(
      "linear_system_consistency", &ADMMSolverStats::linear_system_consistency,
      "History of linear system consistency.")
    .def_ro(
      "delassus_decomposition_update_count", &ADMMSolverStats::delassus_decomposition_update_count,
      "Number of Delassus decomposition updates.");

  // --- ADMMConstraintSolver ---
  nb::class_<ADMMSolver>(
    m, "ADMMConstraintSolver",
    "Alternating Direction Method of Multipliers (ADMM) solver for contact dynamics.")
    .def(nb::init<>(), "Default constructor.")
    .def(nb::init<std::size_t>(), "problem_size"_a, "Constructor with problem dimension.")
    .def(ConstraintSolverBaseVisitor<ADMMSolver>())
    // ADMM-specific
    .def_prop_ro(
      "stats", [](const ADMMSolver & self) -> const ADMMSolverStats & { return self.stats; },
      nb::rv_policy::reference_internal, "Access the statistics of the solver.")
    .def(
      "isValid", &ADMMSolver::isValid,
      "Check if the solver is in a valid state (has solved a constraint problem).")
    // solve() with ConstraintCholeskyDecomposition::DelassusOperatorCholeskyExpression
    .def(
      "solve",
      [](
        ADMMSolver & self, DelassusOperatorCholesky & delassus, ConstVectorRef g,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas, const ADMMSolverSettings & settings,
        ADMMSolverResult & result) -> bool {
        return self.solve(delassus, g, constraint_models, constraint_datas, settings, result);
      },
      "delassus"_a, "g"_a, "constraint_models"_a, "constraint_datas"_a, "settings"_a, "result"_a,
      "Solve the constrained conic problem with given settings and result.")
    // solve() with DelassusOperatorDense
    .def(
      "solve",
      [](
        ADMMSolver & self, DelassusOperatorDense & delassus, ConstVectorRef g,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas, const ADMMSolverSettings & settings,
        ADMMSolverResult & result) -> bool {
        return self.solve(delassus, g, constraint_models, constraint_datas, settings, result);
      },
      "delassus"_a, "g"_a, "constraint_models"_a, "constraint_datas"_a, "settings"_a, "result"_a,
      "Solve the constrained conic problem with given settings and result.")
    // solve() with DelassusOperatorSparse
    .def(
      "solve",
      [](
        ADMMSolver & self, DelassusOperatorSparse & delassus, ConstVectorRef g,
        const ConstraintModelVector & constraint_models,
        const ConstraintDataVector & constraint_datas, const ADMMSolverSettings & settings,
        ADMMSolverResult & result) -> bool {
        return self.solve(delassus, g, constraint_models, constraint_datas, settings, result);
      },
      "delassus"_a, "g"_a, "constraint_models"_a, "constraint_datas"_a, "settings"_a, "result"_a,
      "Solve the constrained conic problem with given settings and result.");
}

void exposeConstraintSolvers(nb::module_ m)
{
  exposePGS(m);
  exposeADMM(m);
}

PINOCCHIO_PYTHON_NAMESPACE_END
