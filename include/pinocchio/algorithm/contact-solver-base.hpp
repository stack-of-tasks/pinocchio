//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_contact_solver_base_hpp__
#define __pinocchio_algorithm_contact_solver_base_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <hpp/fcl/timings.h>
#endif // PINOCCHIO_WITH_HPP_FCL

namespace pinocchio
{

  template<typename _Scalar>
  struct ContactSolverBaseTpl
  {
    typedef _Scalar Scalar;

#ifdef PINOCCHIO_WITH_HPP_FCL
    typedef hpp::fcl::CPUTimes CPUTimes;
    typedef hpp::fcl::Timer Timer;
#endif // PINOCCHIO_WITH_HPP_FCL

    explicit ContactSolverBaseTpl(const int problem_size)
    : problem_size(problem_size)
    , max_it(1000)
    , it(0)
    , absolute_precision(Scalar(1e-6))
    , relative_precision(Scalar(1e-6))
    , absolute_residual(Scalar(-1))
    , relative_residual(Scalar(-1))
#ifdef PINOCCHIO_WITH_HPP_FCL
    , timer(false)
#endif // PINOCCHIO_WITH_HPP_FCL
    {
    }

    /// \brief Returns the size of the problem
    int getProblemSize() const
    {
      return problem_size;
    }

    /// \brief Get the number of iterations achieved by the solver.
    int getIterationCount() const
    {
      return it;
    }

    /// \brief Set the maximum number of iterations.
    void setMaxIterations(const int max_it)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(max_it > 0, "max_it should be greater than 0.");
      this->max_it = max_it;
    }
    /// \brief Get the maximum number of iterations allowed.
    int getMaxIterations() const
    {
      return max_it;
    }

    /// \brief Set the absolute precision for the problem.
    void setAbsolutePrecision(const Scalar absolute_precision)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        absolute_precision >= Scalar(0), "absolute_precision should be positive.");
      this->absolute_precision = absolute_precision;
    }
    /// \brief Get the absolute precision requested.
    Scalar getAbsolutePrecision() const
    {
      return absolute_precision;
    }

    /// \brief Set the relative precision for the problem.
    void setRelativePrecision(const Scalar relative_precision)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        relative_precision >= Scalar(0), "relative_precision should be positive.");
      this->relative_precision = relative_precision;
    }
    /// \brief Get the relative precision requested.
    Scalar getRelativePrecision() const
    {
      return relative_precision;
    }

    /// \brief Returns the value of the absolute residual value corresponding to the contact
    /// complementary conditions.
    Scalar getAbsoluteConvergenceResidual() const
    {
      return absolute_residual;
    }
    /// \brief Returns the value of the relative residual value corresponding to the difference
    /// between two successive iterates (infinity norms).
    Scalar getRelativeConvergenceResidual() const
    {
      return relative_residual;
    }

#ifdef PINOCCHIO_WITH_HPP_FCL
    CPUTimes getCPUTimes() const
    {
      return timer.elapsed();
    }
#endif // PINOCCHIO_WITH_HPP_FCL

  protected:
    /// \brief Size of the problem
    int problem_size;
    ///  \brief Maximum number of iterations.
    int max_it;
    /// \brief Number of iterations needed to achieve convergence.
    int it;
    /// \brief Desired absolute precision.
    Scalar absolute_precision;
    /// \brief Desired relative precision.
    Scalar relative_precision;
    /// \brief Absolule convergence residual value.
    Scalar absolute_residual;
    /// \brief Relative convergence residual value
    Scalar relative_residual;

#ifdef PINOCCHIO_WITH_HPP_FCL
    Timer timer;
#endif // PINOCCHIO_WITH_HPP_FCL

  }; // struct ContactSolverBaseTpl

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_solver_base_hpp__
