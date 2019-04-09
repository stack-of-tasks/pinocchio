//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_algorithm_proximal_hpp__
#define __pinocchio_algorithm_proximal_hpp__

#include <Eigen/Core>

namespace pinocchio
{
  
  ///
  /// \brief Structure containing all the settings paramters for the proximal algorithms.
  ///
  /// \tparam _Scalar Scalar type of the for the regularization and the threshold parameter.
  ///
  /// It contains the threshold, the maximal number of iterations and the regularization factor common to all proximal algorithms.
  ///
  template<typename _Scalar>
  struct ProximalSettingsTpl
  {
    typedef _Scalar Scalar;
    
    /// \brief Default constructor.
    ProximalSettingsTpl()
    : threshold(Eigen::NumTraits<Scalar>::dummy_precision())
    , mu(0.)
    , max_it(1)
    {}
    
    ///
    /// \brief Constructor with all the setting parameters.
    ///
    ProximalSettingsTpl(Scalar threshold,
                        Scalar mu,
                        int max_it)
    : threshold(threshold)
    , mu(mu)
    , max_it(max_it)
    {
      assert(threshold >= 0. && "threshold must be positive");
      assert(mu >= 0. && "mu must be positive");
      assert(max_it >= 1 && "max_it must be greater or equal to 1");
    }
    
    // data
    
    /// \brief Proximal threshold: threshold for two consecutive iterates of the
    /// Proximal algorithms.
    Scalar threshold;
    
    /// \brief Regularization parameter of the Proximal algorithms.
    Scalar mu;
    
    /// \brief Maximal number of iterations.
    int max_it;
    
  };
  
  typedef ProximalSettingsTpl<double> ProximalSettings;
}

#endif // ifndef __pinocchio_algorithm_proximal_hpp__

