//
// Copyright (c) 2016,2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_finite_differences_hxx__
#define __se3_finite_differences_hxx__

#include "pinocchio/multibody/visitor.hpp"

/// @cond DEV

namespace se3
{
  namespace details
  {
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType>
    struct FinitDiffEpsVisitor
    : public fusion::JointVisitorBase< FinitDiffEpsVisitor<Scalar,Options,JointCollectionTpl,TangentVectorType> >
    {
      typedef boost::fusion::vector< TangentVectorType & > ArgsType;
      
      template<typename JointModel>
      static void algo(const JointModelBase<JointModel> & jmodel,
                       const Eigen::MatrixBase<TangentVectorType> & fd_increment)
      {
        jmodel.jointVelocitySelector(EIGEN_CONST_CAST(TangentVectorType,fd_increment))
        .fill(jmodel.finiteDifferenceIncrement());
      }
      
    }; // struct FinitDiffEpsVisitor
    
  } // namespace details
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType
  finiteDifferenceIncrement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    using namespace se3::details;
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::TangentVectorType ReturnType;
    
    ReturnType fd_increment(model.nv);
    typedef FinitDiffEpsVisitor<Scalar,Options,JointCollectionTpl,ReturnType> Algo;
    typename Algo::ArgsType arg(fd_increment);
    for(JointIndex k = 1; k < (JointIndex)model.njoints; ++k)
    {
      Algo::run(model.joints[k],arg);
    }
    
    return fd_increment;
  }
} // namespace se3

/// @endcond

#endif // ifndef __se3_finite_differences_hxx__
