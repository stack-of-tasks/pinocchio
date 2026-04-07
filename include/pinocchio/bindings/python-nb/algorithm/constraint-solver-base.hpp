// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

#include "pinocchio/algorithm/solvers/constraint-solver-base.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<typename Settings>
struct ConstraintSolverSettingsBaseVisitor
: nanobind::def_visitor<ConstraintSolverSettingsBaseVisitor<Settings>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def_rw("max_iterations", &Settings::max_iterations, "Maximum number of solver iterations.")
      .def_rw(
        "absolute_feasibility_tol", &Settings::absolute_feasibility_tol,
        "Absolute tolerance on primal/dual feasibility.")
      .def_rw(
        "relative_feasibility_tol", &Settings::relative_feasibility_tol,
        "Relative tolerance on primal/dual feasibility.")
      .def_rw(
        "absolute_complementarity_tol", &Settings::absolute_complementarity_tol,
        "Absolute tolerance on complementarity (duality gap).")
      .def_rw(
        "relative_complementarity_tol", &Settings::relative_complementarity_tol,
        "Relative tolerance on complementarity (duality gap).")
      .def_rw(
        "solve_ncp", &Settings::solve_ncp,
        "Whether to solve the NCP. If false, the equivalent CCP is solved.")
      .def_rw("measure_timings", &Settings::measure_timings, "Whether to measure solve timings.")
      .def_rw("stat_record", &Settings::stat_record, "Whether to record per-iteration stats.")
      .def("checkValidity", &Settings::checkValidity, "Throw if the settings are not valid.");
  }
}; // struct ConstraintSolverSettingsBaseVisitor

template<typename Result>
struct ConstraintSolverResultBaseVisitor
: nanobind::def_visitor<ConstraintSolverResultBaseVisitor<Result>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def_rw("iterations", &Result::iterations, "Number of solver iterations.")
      .def_rw("converged", &Result::converged, "Whether the solver has converged.")
      .def_ro("primal_feasibility", &Result::primal_feasibility, "Value of the primal feasibility.")
      .def_ro("dual_feasibility", &Result::dual_feasibility, "Value of the dual feasibility.")
      .def_ro("complementarity", &Result::complementarity, "Value of the complementarity.")
      .def(
        "isValid", &Result::isValid,
        "Return True if the result represents a valid (post-solve) state.")
      .def(
        "constraintSize", &Result::constraintSize,
        "Size of quantities related to constraints contained in result (typically constraint "
        "impulses or velocities).")
      .def("reset", &Result::reset, "Reset the result to an invalid (pre-solve) state.")
      .def(
        "retrieveConstraintImpulses",
        [](const Result & self) {
          VectorXs out(static_cast<Eigen::Index>(self.constraintSize()));
          self.retrieveConstraintImpulses(out);
          return out;
        },
        "Return the primal solution (constraint impulses) as a new vector.")
      .def(
        "retrieveConstraintVelocities",
        [](const Result & self) {
          VectorXs out(static_cast<Eigen::Index>(self.constraintSize()));
          self.retrieveConstraintVelocities(out);
          return out;
        },
        "Return the dual solution (constraint velocities) as a new vector.")
      .def(
        "setConstraintImpulseGuess",
        [](Result & self, ConstVectorRef impulse_guess) {
          self.setConstraintImpulseGuess(impulse_guess);
        },
        "impulse_guess"_a, "Set the constraint impulse warmstart guess for the next solve call.")
      .def(
        "clearConstraintImpulseGuess", &Result::clearConstraintImpulseGuess,
        "Clear the constraint impulse warmstart guess.")
      .def(
        "setConstraintVelocityGuess",
        [](Result & self, ConstVectorRef velocity_guess) {
          self.setConstraintVelocityGuess(velocity_guess);
        },
        "velocity_guess"_a, "Set the constraint velocity warmstart guess for the next solve call.")
      .def(
        "clearConstraintVelocityGuess", &Result::clearConstraintVelocityGuess,
        "Clear the constraint velocity warmstart guess.");
  }
}; // struct ConstraintSolverResultBaseVisitor

template<typename Stats>
struct ConstraintSolverStatsBaseVisitor
: nanobind::def_visitor<ConstraintSolverStatsBaseVisitor<Stats>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def_rw("iterations", &Stats::iterations, "Total number of iterations tracked.")
      .def_ro(
        "primal_feasibility", &Stats::primal_feasibility,
        "Per-iteration history of primal feasibility values.")
      .def_ro(
        "dual_feasibility", &Stats::dual_feasibility,
        "Per-iteration history of dual feasibility values.")
      .def_ro(
        "dual_feasibility_ncp", &Stats::dual_feasibility_ncp,
        "Per-iteration history of NCP dual feasibility values.")
      .def_ro(
        "complementarity", &Stats::complementarity,
        "Per-iteration history of complementarity values.")
      .def(
        "size", &Stats::size, "Return the number of iterations tracked (size of history vectors).")
      .def(
        "reserve", &Stats::reserve, "max_iterations"_a,
        "Reserve storage for up to max_iterations iterations.")
      .def("reset", &Stats::reset, "Clear all history and reset the iteration counter to zero.");
  }
}; // struct ConstraintSolverStatsBaseVisitor

template<typename Solver>
struct ConstraintSolverBaseVisitor : nanobind::def_visitor<ConstraintSolverBaseVisitor<Solver>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def("reset", &Solver::reset, "Reset the solver to its initial state.")
      .def(
        "getElapsedTime", &Solver::getElapsedTime,
        "Return last solve call elapsed time in microseconds.");
  }
}; // struct ConstraintSolverBaseVisitor

PINOCCHIO_PYTHON_NAMESPACE_END
