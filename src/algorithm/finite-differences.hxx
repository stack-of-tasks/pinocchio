//
// Copyright (c) 2016,2018 CNRS
//

#ifndef __pinocchio_finite_differences_hxx__
#define __pinocchio_finite_differences_hxx__

#include "pinocchio/multibody/visitor.hpp"

/// @cond DEV

namespace pinocchio
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
        jmodel.jointVelocitySelector(PINOCCHIO_EIGEN_CONST_CAST(TangentVectorType,fd_increment))
        .fill(jmodel.finiteDifferenceIncrement());
      }
      
    }; // struct FinitDiffEpsVisitor
    
  } // namespace details
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType
  finiteDifferenceIncrement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    using namespace pinocchio::details;
    
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
} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_finite_differences_hxx__
