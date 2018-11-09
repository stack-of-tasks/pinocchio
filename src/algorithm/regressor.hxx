//
// Copyright (c) 2018 CNRS
//

#ifndef __pinocchio_regressor_hxx__
#define __pinocchio_regressor_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio
{
  
  namespace regressor
  {
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    inline typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
    computeStaticRegressor(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           DataTpl<Scalar,Options,JointCollectionTpl> & data,
                           const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      assert(model.check(data) && "data is not consistent with model.");
      assert(q.size() == model.nq);
      
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;

      typedef typename Data::Matrix3x Matrix3x;
      typedef typename SizeDepType<4>::ColsReturn<Matrix3x>::Type ColsBlock;
      
      forwardKinematics(model,data,q.derived());
      
      // Computes the total mass of the system
      Scalar mass = Scalar(0);
      for(JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
        mass += model.inertias[(JointIndex)i].mass();
      
      const Scalar mass_inv = Scalar(1)/mass;
      for(JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
      {
        const SE3 & oMi = data.oMi[i];
        ColsBlock sr_cols = data.staticRegressor.template middleCols<4>((Eigen::DenseIndex)(i-1)*4);
        sr_cols.col(0) = oMi.translation();
        sr_cols.template rightCols<3>() = oMi.rotation();
        sr_cols *= mass_inv;
      }
      
      return data.staticRegressor;
    }
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_regressor_hxx__
