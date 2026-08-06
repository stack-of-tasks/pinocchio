//
// Copyright (c) 2022-2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/algorithm/solvers/admm-solver.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/algorithm/solvers/admm-solver.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  template<
    typename DelassusDerived,
    typename VectorLike,
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename ConstraintData,
    typename ConstraintDataAllocator>
  bool ADMMConstraintSolverTpl<_Scalar, _Options>::solveImpl(
    DelassusOperatorBase<DelassusDerived> & delassus,
    const Eigen::MatrixBase<VectorLike> & g,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const ADMMSolverSettings & settings,
    ADMMSolverResult & result)
  {
    // for easier access
    ADMMSolverResult & res = result;
    ADMMSolverWorkspace & ws = m_workspace;
    DelassusDerived & G = delassus.derived();

    // Configure/reset workspace, stats and result
    // note: the order matters as workspace is initialized using
    // optional warmstarts contained in result.
    const Eigen::Index np = G.rows();
    const std::size_t problem_size = static_cast<std::size_t>(np);
    assert(G.cols() == np);
    assert(g.size() == np);
    assert(residualSize(constraint_models) == np);

    // -- check if settings are valid
    settings.checkValidity();

    // -- reset workspace
    ws.resize(problem_size, settings.lanczos_size, settings.anderson_capacity);
    ws.reset();
    assert(ws.problem_size == problem_size);
    assert(ws.x.size() == np);

    // -- reset per-iteration statistics
    stats.reset();
    if (settings.stat_record)
    {
      stats.reserve(settings.max_iterations);
    }

    // -- retrieve warmstart from results, then reset results
    retrievePrimalDualGuess(delassus, g, constraint_models, constraint_datas, settings, res, ws);
    retrieveRhoGuess(delassus, settings, res, ws);
    switch (settings.admm_proximal_rule)
    {
    case (ADMMProximalRule::MANUAL):
      ws.mu_prox = settings.mu_prox;
      break;
    case (ADMMProximalRule::AUTOMATIC):
      ws.mu_prox = ws.rho;
      break;
    }
    res.resize(problem_size);
    res.reset();
    assert(res.isValid() == false);
    assert(res.problem_size == problem_size);
    assert(res.iterations == 0);

    // -- init of internals done - the solver is now marked as reset
    m_is_valid = false;

    if (settings.measure_timings)
    {
      timerStart();
    }

    // Check NCP/CCP conditions. If they are satisfied, don't run the solver.
    // -- always primaly feasible as y is projected onto the constraints
    // (this has been done in `retrievePrimalDualGuess`)
    res.primal_feasibility = Scalar(0);

    // -- dual feasibility
    G.applyOnTheRight(ws.y, ws.rhs, false /* without damping */);
    ws.rhs += g;
    if (settings.solve_ncp)
    {
      internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.rhs, ws.desaxce);
      ws.rhs += ws.desaxce;
    }
    else
    {
      ws.desaxce.setZero();
    }
    internal::computeDualConstraintSetProjection(
      constraint_models, constraint_datas, ws.rhs, ws.tmp);
    ws.tmp -= ws.rhs;
    res.dual_feasibility = ws.tmp.template lpNorm<Eigen::Infinity>();

    // -- complementarity
    internal::computeConicComplementarity(
      constraint_models, constraint_datas, ws.rhs, ws.y, res.complementarity);

    bool abs_prec_reached = false;
    bool rel_prec_reached = false;
    if (
      check_expression_if_real<Scalar, false>(
        res.complementarity <= settings.absolute_complementarity_tol)
      && check_expression_if_real<Scalar, false>(
        res.dual_feasibility <= settings.absolute_feasibility_tol))
    {
      abs_prec_reached = true;
      ws.z = ws.rhs; // store dual solution
    }

    if (!abs_prec_reached)
    {
      // Setup ADMM update rules:
      // Before running ADMM, we compute the largest and smallest eigenvalues of delassus in order
      // to be able to use a spectral update rule for the proximal parameter (rho)
      ADMMUpdateRuleContainer admm_update_rule_container;
      switch (settings.admm_update_rule)
      {
      case (ADMMUpdateRule::SPECTRAL): {
        if (ws.delassus_largest_eigenvalue.has_value() == false)
        {
          // largest eigenvalue has not yet been computed, we compute it
          ws.delassus_largest_eigenvalue = computeDelassusLargestEigenvalue(G, ws);
        }
        admm_update_rule_container.spectral_rule = ADMMSpectralUpdateRule(
          settings.ratio_primal_dual,              //
          ws.delassus_largest_eigenvalue.value(),  //
          ws.delassus_smallest_eigenvalue.value(), //
          settings.spectral_rho_power_factor);
        break;
      }
      case (ADMMUpdateRule::OSQP):
        admm_update_rule_container.osqp_rule =
          ADMMOSQPUpdateRule(settings.ratio_primal_dual, Scalar(1e-8));
        break;
      case (ADMMUpdateRule::LINEAR):
        admm_update_rule_container.linear_rule =
          ADMMLinearUpdateRule(settings.ratio_primal_dual, settings.linear_update_rule_factor);
        break;
      case (ADMMUpdateRule::CONSTANT):
        break;
      }

      // Update the decomposition of the Delassus
      Scalar prox_value = settings.tau_prox * ws.mu_prox + settings.tau * ws.rho;
      G.updateDamping(prox_value);
      G.updateDecomposition();
      Scalar old_prox_value = prox_value;
      ws.delassus_decomposition_update_count++;

      // End of Initialization phase
      ws.x_anderson = ws.x;
      ws.z_anderson = ws.z;
      ws.anderson_history.clear();
      Scalar anderson_primal_feasibility;
      Scalar anderson_previous_primal_feasibility = std::numeric_limits<Scalar>::max();

      Scalar dx_norm = std::numeric_limits<Scalar>::quiet_NaN();
      Scalar dy_norm = std::numeric_limits<Scalar>::quiet_NaN();
      Scalar dz_norm = std::numeric_limits<Scalar>::quiet_NaN();
      const Scalar g_norm_inf = g.template lpNorm<Eigen::Infinity>();
      Scalar x_norm_inf = ws.x.template lpNorm<Eigen::Infinity>();
      Scalar y_norm_inf = ws.y.template lpNorm<Eigen::Infinity>();
      Scalar z_norm_inf = ws.z.template lpNorm<Eigen::Infinity>();
      Scalar x_previous_norm_inf = x_norm_inf;
      Scalar y_previous_norm_inf = y_norm_inf;
      Scalar z_previous_norm_inf = z_norm_inf;

      res.iterations = 0;
      std::size_t it_since_last_rho_update = 0;
      for (; res.iterations <= settings.max_iterations;
           ++res.iterations, ++it_since_last_rho_update)
      {
        // Fit the Anderson acceleration to compute accelerated x and y iterates
        if (res.iterations > 1)
        {
          if (ws.anderson_history.capacity() > 0)
          {
            ws.anderson_history.push_back(ws.x, ws.z, ws.z - ws.z_previous);
          }

          if (
            ws.anderson_history.capacity() == 0 //
            || ws.anderson_history.size() < ws.anderson_history.capacity())
          {
            ws.x_anderson = ws.x;
            ws.z_anderson = ws.z;
          }
          else
          {
            ws.anderson_history.fit();
            ws.anderson_history.getAcceleratedIterates(ws.x_anderson, ws.z_anderson);
          }
        }

        // store previous iterates
        // note: when Anderson capacity is < 2, x_anderson_ = x_
        ws.x_previous = ws.x_anderson;
        ws.y_previous = ws.y;
        ws.z_previous = ws.z_anderson;
        res.complementarity = Scalar(0);

        // y-update, using Anderson iterate.
        // If update is worse in terms of primal feas, it is rejected and the default
        // ADMM iterates are used to compute the y-update.
        {
          PINOCCHIO_TRACY_ZONE_SCOPED_N(
            "ADMMConstraintSolverTpl::solve - loop computeConstraintSetProjection");
          ws.tmp = ws.x_anderson - ws.z_anderson / (settings.tau * ws.rho);
          internal::computeConstraintSetProjection(
            constraint_models, constraint_datas, ws.tmp, ws.y);

          ws.anderson_primal_feasibility_vector = ws.x_anderson - ws.y;
          anderson_primal_feasibility =
            ws.anderson_primal_feasibility_vector.template lpNorm<Eigen::Infinity>();

          if (ws.anderson_history.capacity() > 1 && res.iterations > 1)
          {
            if (
              anderson_primal_feasibility >= anderson_previous_primal_feasibility //
              && ws.anderson_history.size() == ws.anderson_history.capacity())
            {
              // Reject Anderson iterate, accept default ADMM iterate instead.
              // Reset Anderson acceleration.
              ws.x_previous = ws.x;
              ws.z_previous = ws.z;
              ws.tmp = ws.x_previous - ws.z_previous / (settings.tau * ws.rho);
              internal::computeConstraintSetProjection(
                constraint_models, constraint_datas, ws.tmp, ws.y);

              ws.anderson_history.clear();

              ws.anderson_primal_feasibility_vector = ws.x_previous - ws.y;
              anderson_primal_feasibility =
                ws.anderson_primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
            }
          }
        }
        anderson_previous_primal_feasibility = anderson_primal_feasibility;

        if (settings.solve_ncp)
        {
          // s-update
          internal::computeDeSaxeCorrection(
            constraint_models, constraint_datas, ws.z_previous, ws.desaxce);
        }
        else
        {
          ws.desaxce.setZero();
        }

        // default (non-accelerated) x-update
        {
          PINOCCHIO_TRACY_ZONE_SCOPED_N("ADMMConstraintSolverTpl::solve - loop solveInPlace");
          ws.rhs =
            -(g + ws.desaxce - (ws.rho * settings.tau) * ws.y
              - (ws.mu_prox * settings.tau_prox) * ws.x_previous - ws.z_previous);
          ws.x = ws.rhs;
          G.solveInPlace(ws.x);
        }
        if (settings.stat_record)
        {
          G.applyOnTheRight(ws.x, ws.tmp);
          Scalar linear_system_residual = (ws.tmp - ws.rhs).template lpNorm<Eigen::Infinity>();
          stats.linear_system_residual.push_back(linear_system_residual);

          G.solveInPlace(ws.tmp);
          Scalar linear_system_consistency = (ws.tmp - ws.x).template lpNorm<Eigen::Infinity>();
          stats.linear_system_consistency.push_back(linear_system_consistency);
        }

        // default (non-accelerated) z-update
        ws.tmp = ws.z_previous - (settings.tau * ws.rho) * (ws.x - ws.y);
        ws.z.noalias() =
          settings.dual_momentum * ws.z_previous + (Scalar(1) - settings.dual_momentum) * ws.tmp;

        // check termination criteria
        ws.primal_feasibility_vector = ws.x - ws.y;

        {
          auto & dx = ws.tmp;
          dx = ws.x - ws.x_previous;
          dx_norm = dx.template lpNorm<Eigen::Infinity>(); // check relative progress on x
          ws.dual_feasibility_vector = dx;
        }

        {
          auto & dy = ws.tmp;
          dy = ws.y - ws.y_previous;
          dy_norm = dy.template lpNorm<Eigen::Infinity>(); // check relative progress on y
        }

        {
          auto & dz = ws.tmp;
          dz = ws.z - ws.z_previous;
          dz_norm = dz.template lpNorm<Eigen::Infinity>(); // check relative progress on z
        }

        // compute primal/dual feasibility and complementarity
        // --> these are used to check convergence of the algo
        res.primal_feasibility = ws.primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
        res.dual_feasibility = ws.dual_feasibility_vector.template lpNorm<Eigen::Infinity>();
        res.dual_feasibility =
          math::max(ws.mu_prox * settings.tau_prox, ws.rho * settings.tau) * res.dual_feasibility;
        internal::computeConicComplementarity(
          constraint_models, constraint_datas, ws.z, ws.y, res.complementarity);

        if (settings.stat_record)
        {
          G.applyOnTheRight(ws.y, ws.rhs);
          ws.rhs += g - prox_value * ws.y;
          if (settings.solve_ncp)
          {
            internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.rhs, ws.tmp);
            ws.rhs += ws.tmp;
          }

          internal::computeDualConstraintSetProjection(
            constraint_models, constraint_datas, ws.rhs, ws.tmp);
          ws.rhs -= ws.tmp;

          Scalar dual_feasibility_ncp = ws.rhs.template lpNorm<Eigen::Infinity>();

          stats.primal_feasibility.push_back(res.primal_feasibility);
          stats.dual_feasibility.push_back(res.dual_feasibility);
          stats.dual_feasibility_ncp.push_back(dual_feasibility_ncp);
          stats.complementarity.push_back(res.complementarity);
          stats.rho.push_back(ws.rho);
          stats.mu_prox.push_back(ws.mu_prox);
          stats.anderson_size.push_back(ws.anderson_history.size());
        }

        // Checking stopping residual
        x_norm_inf = ws.x.template lpNorm<Eigen::Infinity>();
        y_norm_inf = ws.y.template lpNorm<Eigen::Infinity>();
        z_norm_inf = ws.z.template lpNorm<Eigen::Infinity>();
        // -- absolute check
        if (
          check_expression_if_real<Scalar, false>(
            res.complementarity <= settings.absolute_complementarity_tol)
          && check_expression_if_real<Scalar, false>(
            res.dual_feasibility
            <= settings.absolute_feasibility_tol
                 + settings.relative_feasibility_tol * math::max(g_norm_inf, z_norm_inf))
          && check_expression_if_real<Scalar, false>(
            res.primal_feasibility
            <= settings.absolute_feasibility_tol
                 + settings.relative_feasibility_tol * math::max(x_norm_inf, y_norm_inf)))
        {
          abs_prec_reached = true;
        }
        else
        {
          abs_prec_reached = false;
        }

        // -- relative check
        if (
          check_expression_if_real<Scalar, false>(
            dx_norm
            <= settings.relative_feasibility_tol * math::max(x_norm_inf, x_previous_norm_inf))
          && check_expression_if_real<Scalar, false>(
            dy_norm
            <= settings.relative_feasibility_tol * math::max(y_norm_inf, y_previous_norm_inf))
          && check_expression_if_real<Scalar, false>(
            dz_norm
            <= settings.relative_feasibility_tol * math::max(z_norm_inf, z_previous_norm_inf)))
        {
          rel_prec_reached = true;
        }
        else
        {
          rel_prec_reached = false;
        }

        if (abs_prec_reached || rel_prec_reached)
          break;

        // update rho if needed
        if (
          ws.delassus_decomposition_update_count < settings.max_delassus_decomposition_updates
          && it_since_last_rho_update >= settings.rho_min_update_frequency)
        {
          // Apply rho according to the primal_dual_ratio
          Scalar new_rho = ws.rho;
          switch (settings.admm_update_rule)
          {
          case (ADMMUpdateRule::SPECTRAL):
            admm_update_rule_container.spectral_rule.eval(
              res.primal_feasibility, res.dual_feasibility, new_rho);
            break;
          case (ADMMUpdateRule::OSQP):
            admm_update_rule_container.osqp_rule.eval(
              res.primal_feasibility, res.dual_feasibility, new_rho);
            break;
          case (ADMMUpdateRule::LINEAR):
            admm_update_rule_container.linear_rule.eval(
              res.primal_feasibility, res.dual_feasibility, new_rho);
            break;
          case (ADMMUpdateRule::CONSTANT):
            break;
          }

          // clamp rho a second time
          new_rho = math::max(math::min(new_rho, settings.rho_max), settings.rho_min);

          // apply a momentum strategy on rho defined by:
          new_rho = std::pow(ws.rho, settings.rho_momentum)
                    * std::pow(new_rho, Scalar(1) - settings.rho_momentum);

          // clamp rho a second time in case the new values is outside the bounds
          new_rho = math::max(math::min(new_rho, settings.rho_max), settings.rho_min);

          bool update_delassus_factorization = false;
          if (new_rho == ws.rho)
          { // No change of rho, so need to redo a factorization
            update_delassus_factorization = false;
          }
          else if (
            new_rho >= settings.rho_update_ratio * ws.rho
            || ws.rho >= settings.rho_update_ratio * new_rho)
          { // sufficient change of the rho value
            ws.rho = new_rho;
            switch (settings.admm_proximal_rule)
            {
            case (ADMMProximalRule::MANUAL):
              // don't update the mu_prox
              break;
            case (ADMMProximalRule::AUTOMATIC):
              ws.mu_prox = ws.rho;
              break;
            }
            it_since_last_rho_update = 0;
            update_delassus_factorization = true;
          }

          // Account for potential update of rho
          if (update_delassus_factorization)
          {
            prox_value = settings.tau_prox * ws.mu_prox + settings.tau * ws.rho;
            if (old_prox_value != prox_value)
            {
              PINOCCHIO_TRACY_ZONE_SCOPED_N("ADMMConstraintSolverTpl::solve - loop updateDamping");
              G.updateDamping(prox_value);
              G.updateDecomposition();
              ws.delassus_decomposition_update_count++;
              old_prox_value = prox_value;
            }
          }
        }

        x_previous_norm_inf = x_norm_inf;
        y_previous_norm_inf = y_norm_inf;
        z_previous_norm_inf = z_norm_inf;

      } // end ADMM main for loop

      // Save values of spectral update rule
      if (settings.admm_update_rule == ADMMUpdateRule::SPECTRAL)
      {
        ws.spectral_rho_power = ADMMSpectralUpdateRule::computeRhoPower(
          ws.delassus_largest_eigenvalue.value(),  //
          ws.delassus_smallest_eigenvalue.value(), //
          ws.rho);
      }
    }

    if (settings.measure_timings)
    {
      timerStop();
    }

    if (settings.stat_record)
    {
      stats.iterations = res.iterations;
      stats.delassus_decomposition_update_count = ws.delassus_decomposition_update_count;
    }

    res.x = ws.x;
    res.y = ws.y;
    res.z = ws.z;
    res.desaxce = ws.desaxce;
    res.rho = ws.rho;
    res.spectral_rho_power = ws.spectral_rho_power;
    res.mu_prox = ws.mu_prox;
    res.delassus_decomposition_update_count = ws.delassus_decomposition_update_count;
    res.converged = abs_prec_reached || rel_prec_reached;
    res.unsafe().makeValid();

    // the solver has run, we mark it as valid
    m_is_valid = true;

    return res.converged;
  }

  template<typename Scalar, int Options>
  template<typename DelassusDerived>
  Scalar ADMMConstraintSolverTpl<Scalar, Options>::computeDelassusLargestEigenvalue(
    const DelassusOperatorBase<DelassusDerived> & delassus, ADMMSolverWorkspace & workspace)
  {
    const DelassusDerived & G = delassus.derived();
    Scalar L = Scalar(-1);
    if (workspace.problem_size > 1)
    {
      PINOCCHIO_TRACY_ZONE_SCOPED_N("ADMMConstraintSolverTpl::solve - lanczos");
      workspace.lanczos_decomposition.compute(G);
      L = ::pinocchio::computeLargestEigenvalue(workspace.lanczos_decomposition.Ts(), Scalar(1e-8));
#ifndef NDEBUG
      const bool enforce_symmetry = true;
      MatrixXs delassus = G.matrix(enforce_symmetry);
      Eigen::SelfAdjointEigenSolver<MatrixXs> solver(delassus);
      VectorXs eigvals = solver.eigenvalues();
      Scalar true_L = eigvals.maxCoeff();
      PINOCCHIO_UNUSED_VARIABLE(true_L);
      //          if (true_m > 0)
      //          {
      //            assert(
      //              math::fabs((true_m - m) / math::max(true_m, m)) < 0.01
      //              && "true_m and m are too far apart.");
      //          }
      // assert(
      //   math::fabs((true_L - L) / math::max(true_L, L)) < 0.01
      //   && "true_L and L are too far apart.");
#endif // NDEBUG
    }
    else
    {
      typedef Eigen::Matrix<Scalar, 1, 1> Vector1;
      Vector1 Gvec;
      G.applyOnTheRight(Vector1(1), Gvec, false /*no damping*/);
      L = Gvec.coeff(0);
    }

    assert(L > 0 && "L must be positive.");
    return L;
  }

  template<typename _Scalar, int _Options>
  template<
    typename DelassusDerived,
    typename VectorLike,
    typename ConstraintModel,
    typename ConstraintModelAllocator,
    typename ConstraintData,
    typename ConstraintDataAllocator>
  void ADMMConstraintSolverTpl<_Scalar, _Options>::retrievePrimalDualGuess(
    DelassusOperatorBase<DelassusDerived> & delassus,
    const Eigen::MatrixBase<VectorLike> & g,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    const std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    const ADMMSolverSettings & settings,
    const ADMMSolverResult & result,
    ADMMSolverWorkspace & workspace)
  {
    // for easier access
    const ADMMSolverResult & res = result;
    DelassusDerived & G = delassus.derived();
    ADMMSolverWorkspace & ws = workspace;

    const Scalar min_compliance = G.getCompliance().minCoeff();
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      min_compliance >= Scalar(0), "compliance should be a positive vector.");

    // initialize De Saxé shift to 0
    // for the CCP, there is no shift.
    // for the NCP, the shift will be initialized using z.
    ws.desaxce.setZero();

    // set initial damping of the delassus to the proximal value and get smallest possible
    // eigenvalue of the problem.
    ws.mu_prox = settings.mu_prox;
    ws.delassus_smallest_eigenvalue = min_compliance + ws.mu_prox;
    G.updateDamping(ws.mu_prox);
    G.updateDecomposition();
    ws.delassus_decomposition_update_count++;

    // check if primal/dual have been given
    bool has_impulse_guess = res.impulse_guess.has_value();
    PINOCCHIO_THROW_PRETTY_IF(
      has_impulse_guess && (res.impulse_guess.value().size() != ws.x.size()), std::runtime_error,
      "Impulse guess given to ADMM is of incorrect size.");
    if (has_impulse_guess)
    {
      // primal guess given but we need to check for size
      if (res.impulse_guess.value().size() != ws.x.size())
      {
        has_impulse_guess = false;
      }
    }

    bool has_velocity_guess = res.velocity_guess.has_value();
    PINOCCHIO_THROW_PRETTY_IF(
      has_velocity_guess && (res.velocity_guess.value().size() != ws.z.size()), std::runtime_error,
      "Velocity guess given to ADMM is of incorrect size.");
    if (has_velocity_guess)
    {
      if (res.velocity_guess.value().size() != ws.z.size())
      {
        has_velocity_guess = false;
      }
    }

    // Initialization of the primal/dual variables.
    // If both primal and dual guesses are given, the solver uses both.
    // If one is given but not the other, the solver will compute the missing one using the given
    // one.
    if (has_impulse_guess)
    {
      if (has_velocity_guess)
      {
        ws.z = res.velocity_guess.value();
        if (settings.solve_ncp)
        {
          // Add De Saxé shift
          internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.z, ws.desaxce);
          ws.z += ws.desaxce;
        }
        ws.x = res.impulse_guess.value();
        internal::computeConstraintSetProjection(constraint_models, constraint_datas, ws.x, ws.y);
      }
      else
      {
        // Warm-start dual variable using primal guess
        ws.x = res.impulse_guess.value();
        internal::computeConstraintSetProjection(constraint_models, constraint_datas, ws.x, ws.y);
        G.applyOnTheRight(ws.y, ws.z, false /*no damping*/);
        ws.z.noalias() += g;
        if (settings.solve_ncp)
        {
          // Add De Saxé shift
          internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.z, ws.desaxce);
          ws.z += ws.desaxce;
        }
      }
    }
    else
    {
      if (has_velocity_guess)
      {
        // Warm-start primal variable using dual guess
        ws.z = res.velocity_guess.value();
        if (settings.solve_ncp)
        {
          internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.z, ws.desaxce);
          ws.z += ws.desaxce;
        }
        ws.x = ws.z - g - ws.desaxce;
        G.solveInPlace(ws.x);
        internal::computeConstraintSetProjection(constraint_models, constraint_datas, ws.x, ws.y);
        // ws.y.setZero();
      }
      else
      {
        ws.x.setZero();
        ws.y.setZero();
        ws.z = g;
        if (settings.solve_ncp)
        {
          internal::computeDeSaxeCorrection(constraint_models, constraint_datas, ws.z, ws.desaxce);
          ws.z += ws.desaxce;
        }
      }
    }

    // sanity checks
    const Eigen::Index np = G.rows();
    PINOCCHIO_CHECK_ARGUMENT_SIZE(ws.x.size(), np);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(ws.y.size(), np);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(ws.z.size(), np);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(ws.desaxce.size(), np);
  }

  template<typename _Scalar, int _Options>
  template<typename DelassusDerived>
  void ADMMConstraintSolverTpl<_Scalar, _Options>::retrieveRhoGuess(
    const DelassusOperatorBase<DelassusDerived> & delassus,
    const ADMMSolverSettings & settings,
    const ADMMSolverResult & result,
    ADMMSolverWorkspace & workspace)
  {
    // for easier access
    const DelassusDerived & G = delassus.derived();
    const ADMMSolverResult & res = result;
    ADMMSolverWorkspace & ws = workspace;

    std::optional<Scalar> rho_init = settings.rho_init;
    Scalar spectral_rho_power_init = settings.spectral_rho_power_init;
    if (settings.warmstart_rho_with_previous_result && res.isValid())
    {
      // override rho_init with previous result's rho value
      rho_init = res.rho;
      spectral_rho_power_init = res.spectral_rho_power;
    }

    // init workspace's rho parameters
    ws.spectral_rho_power = spectral_rho_power_init;
    if (rho_init)
    {
      ws.rho = rho_init.value();
    }
    else
    {
      // compute rho with spectral rule
      assert(ws.delassus_smallest_eigenvalue.has_value() == true);
      assert(ws.delassus_largest_eigenvalue.has_value() == false);
      ws.delassus_largest_eigenvalue = computeDelassusLargestEigenvalue(G, ws);
      if (std::isnan(ws.delassus_largest_eigenvalue.value()))
      {
        ws.delassus_largest_eigenvalue = Scalar(1);
      }
      // TODO: change order of largest/smallest
      ws.rho = ADMMSpectralUpdateRule::computeRho(
        ws.delassus_largest_eigenvalue.value(),  //
        ws.delassus_smallest_eigenvalue.value(), //
        ws.spectral_rho_power);
    }
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ws.rho >= 0, "rho should be positive.");

    // clamp the rho
    ws.rho = math::max(math::min(ws.rho, settings.rho_max), settings.rho_min);
  }

} // namespace pinocchio

#ifdef PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #ifndef PINOCCHIO_SKIP_ALGORITHM_SOLVERS

namespace pinocchio
{

  // -------------------------------------------------------------------------
  // Struct instantiations
  // -------------------------------------------------------------------------

  extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ADMMSolverSettingsTpl<context::Scalar>;

  extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ADMMSolverStatsTpl<context::Scalar>;

  extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ADMMSolverResultTpl<context::Scalar, context::Options>;

  namespace internal
  {
    extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
      ADMMSolverWorkspaceTpl<context::Scalar, context::Options>;
  } // namespace internal

  extern template struct PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    ADMMConstraintSolverTpl<context::Scalar, context::Options>;

  // -------------------------------------------------------------------------
  // solveImpl() with DelassusOperatorDense + default constraint collection
  // -------------------------------------------------------------------------

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
  ADMMConstraintSolverTpl<context::Scalar, context::Options>::solveImpl<
    DelassusOperatorDenseTpl<context::Scalar, context::Options>,
    context::VectorXs,
    ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>,
    ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>>(
    DelassusOperatorBase<DelassusOperatorDenseTpl<context::Scalar, context::Options>> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const std::vector<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const std::vector<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const ADMMSolverSettingsTpl<context::Scalar> &,
    ADMMSolverResultTpl<context::Scalar, context::Options> &);

  // -------------------------------------------------------------------------
  // solveImpl() with DelassusOperatorCholeskyExpression + default constraint collection
  // -------------------------------------------------------------------------

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
  ADMMConstraintSolverTpl<context::Scalar, context::Options>::solveImpl<
    DelassusOperatorCholeskyExpressionTpl<
      ConstraintCholeskyDecompositionTpl<context::Scalar, context::Options>>,
    context::VectorXs,
    ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>,
    ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>>(
    DelassusOperatorBase<DelassusOperatorCholeskyExpressionTpl<
      ConstraintCholeskyDecompositionTpl<context::Scalar, context::Options>>> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const std::vector<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const std::vector<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const ADMMSolverSettingsTpl<context::Scalar> &,
    ADMMSolverResultTpl<context::Scalar, context::Options> &);

  // -------------------------------------------------------------------------
  // solveImpl() with DelassusOperatorRigidBodySystems + default constraint collection
  // -------------------------------------------------------------------------

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
  ADMMConstraintSolverTpl<context::Scalar, context::Options>::solveImpl<
    DelassusOperatorRigidBodySystemsTpl<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::reference_wrapper>,
    context::VectorXs,
    ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>,
    ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
    std::allocator<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>>(
    DelassusOperatorBase<DelassusOperatorRigidBodySystemsTpl<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::reference_wrapper>> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const std::vector<
      ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const std::vector<
      ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>,
      std::allocator<
        ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>>> &,
    const ADMMSolverSettingsTpl<context::Scalar> &,
    ADMMSolverResultTpl<context::Scalar, context::Options> &);

} // namespace pinocchio

  #endif // ifndef PINOCCHIO_SKIP_ALGORITHM_SOLVERS

#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
