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
  typedef std::size_t Index;
  typedef Index JointIndex;
  typedef Index GeomIndex;
  typedef Index FrameIndex;
  typedef Index PairIndex;
  
  template<typename Scalar, int Options=0> struct FrameTpl;
  typedef FrameTpl<double> Frame;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct ModelTpl;
  typedef ModelTpl<double> Model;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct DataTpl;
  typedef DataTpl<double> Data;
  
  struct GeometryModel;
  struct GeometryData;
  
  enum ReferenceFrame
  {
    WORLD = 0,
    LOCAL = 1
  };

  // Forward declaration needed for Model::check
  template<class D> struct AlgorithmCheckerBase;

} // namespace pinocchio

#endif // #ifndef __pinocchio_multibody_fwd_hpp__
