//
// Copyright (c) 2017-2018 CNRS
//

#ifndef __pinocchio_multibody_fwd_hpp__
#define __pinocchio_multibody_fwd_hpp__

#include "pinocchio/fwd.hpp"

# include <cstddef> // std::size_t
#include "pinocchio/multibody/joint/fwd.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options=0> struct FrameTpl;

  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;

  /**
   * \addtogroup multibody
   * @{
   */

  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  typedef Index PairIndex;
  
  typedef FrameTpl<double> Frame;
  
  typedef ModelTpl<double> Model;
  typedef DataTpl<double> Data;
  
  struct GeometryModel;
  struct GeometryData;
  
  enum ReferenceFrame
  {
    WORLD = 0,
    LOCAL = 1
  };

  /**
   * @}
   */
  // end of group multibody

  // Forward declaration needed for Model::check
  template<class D> struct AlgorithmCheckerBase;

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_fwd_hpp__
