//
// Copyright (c) 2019-2022 INRIA
//

#ifndef __pinocchio_algorithm_proximal_hpp__
#define __pinocchio_algorithm_proximal_hpp__

#include <Eigen/Core>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  ///
  /// \brief Structure containing all the settings parameters for the proximal algorithms.
  ///
  ///  \tparam _Scalar Scalar type of the for the regularization and the accuracy parameter.
  ///
  /// It contains the accuracy, the maximal number of iterations and the regularization factor
  /// common to all proximal algorithms.
  ///
  template<typename _Scalar>
  struct ProximalSettingsTpl
  {
    typedef _Scalar Scalar;

    /// \brief Default constructor.
    ProximalSettingsTpl()
    : absolute_accuracy(Eigen::NumTraits<Scalar>::dummy_precision())
    , relative_accuracy(Eigen::NumTraits<Scalar>::dummy_precision())
    , mu(0)
    , max_iter(1)
    , absolute_residual(-1.)
    , relative_residual(-1.)
    , iter(0)
    {
    }

    ///
    /// \brief Constructor with all the setting parameters.
    ///
    ProximalSettingsTpl(const Scalar accuracy, const Scalar mu, const int max_iter)
    : absolute_accuracy(accuracy)
    , relative_accuracy(accuracy)
    , mu(mu)
    , max_iter(max_iter)
    , absolute_residual(-1.)
    , relative_residual(-1.)
    , iter(0)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(accuracy >= 0.) && "Accuracy must be positive.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(mu >= 0.) && "mu must be positive");
      assert(max_iter >= 1 && "max_iter must be greater or equal to 1");
    }

    ///
    /// \brief Constructor with all the setting parameters.
    ///
    ProximalSettingsTpl(
      const Scalar absolute_accuracy,
      const Scalar relative_accuracy,
      const Scalar mu,
      const int max_iter)
    : absolute_accuracy(absolute_accuracy)
    , relative_accuracy(relative_accuracy)
    , mu(mu)
    , max_iter(max_iter)
    , absolute_residual(-1.)
    , relative_residual(-1.)
    , iter(0)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(absolute_accuracy >= 0.)
        && "Absolute accuracy must be positive.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(relative_accuracy >= 0.)
        && "Relative accuracy must be positive.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        check_expression_if_real<Scalar>(mu >= 0.) && "mu must be positive");
      assert(max_iter >= 1 && "max_iter must be greater or equal to 1");
    }

    // data

    /// \brief Absolute proximal accuracy.
    Scalar absolute_accuracy;

    /// \brief Relative proximal accuracy between two iterates.
    Scalar relative_accuracy;

    /// \brief Regularization parameter of the proximal algorithm.
    Scalar mu;

    /// \brief Maximal number of iterations.
    int max_iter;

    // data that can be modified by the algorithm

    /// \brief Absolute residual.
    Scalar absolute_residual;

    /// \brief Relatice residual  between two iterates.
    Scalar relative_residual;

    /// \brief Total number of iterations of the algorithm when it has converged or reached the
    /// maximal number of allowed iterations.
    int iter;
  };

} // namespace pinocchio

#if PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  #include "pinocchio/algorithm/proximal.txx"
#endif // PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION

#endif // ifndef __pinocchio_algorithm_proximal_hpp__
