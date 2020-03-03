//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_algorithm_proximal_hpp__
#define __pinocchio_algorithm_proximal_hpp__

#include <Eigen/Core>

namespace pinocchio
{
  
  ///
  /// \brief Structure containing all the settings paramters for the proximal algorithms.
  ///
  /// \tparam _Scalar Scalar type of the for the regularization and the accuracy parameter.
  ///
  /// It contains the accuracy, the maximal number of iterations and the regularization factor common to all proximal algorithms.
  ///
  template<typename _Scalar>
  struct ProximalSettingsTpl
  {
    typedef _Scalar Scalar;
    
    /// \brief Default constructor.
    ProximalSettingsTpl()
    : accuracy(Eigen::NumTraits<Scalar>::dummy_precision())
    , mu(10.)
    , max_iter(10)
    {}
    
    ///
    /// \brief Constructor with all the setting parameters.
    ///
    ProximalSettingsTpl(Scalar accuracy,
                        Scalar mu,
                        int max_iter)
    : accuracy(accuracy)
    , mu(mu)
    , max_iter(max_iter)
    {
      assert(accuracy >= 0. && "accuracy must be positive");
      assert(mu >= 0. && "mu must be positive");
      assert(max_iter >= 1 && "max_iter must be greater or equal to 1");
    }
    
    // data
    
    /// \brief Minimal proximal accuracy required for primal and dual feasibility
    Scalar accuracy;
    
    /// \brief Regularization parameter of the Proximal algorithms.
    Scalar mu;
    
    /// \brief Maximal number of iterations.
    int max_iter;
    
  };
  
  typedef ProximalSettingsTpl<double> ProximalSettings;
}

#endif // ifndef __pinocchio_algorithm_proximal_hpp__

