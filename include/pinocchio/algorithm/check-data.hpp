//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_algorithm_check_data_hpp__
#define __pinocchio_algorithm_check_data_hpp__

#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{

  /// Check the validity of data wrt to model, in particular if model has been modified.
  ///
  /// \param[in] model reference model
  /// \param[in] data corresponding data
  ///
  /// \returns True if data is valid wrt model.
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  bool checkData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data);

} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/check-data.hxx"

#endif // __pinocchio_algorithm_check_data_hpp__
