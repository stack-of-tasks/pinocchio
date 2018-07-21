//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_joint_configuration_hxx__
#define __se3_joint_configuration_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <cmath>

/* --- Details -------------------------------------------------------------------- */
namespace se3
{
  template<typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<JointCollection> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return integrate<LieGroupMap,JointCollection,ConfigVectorType,TangentVectorType>(model, q.derived(), v.derived());
  }

  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorType, typename TangentVectorType>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<JointCollection> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef typename EIGEN_PLAIN_TYPE(ConfigVectorType) ReturnType;
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType res(model.nq);
    
    typedef IntegrateStep<LieGroup_t,ConfigVectorType,TangentVectorType,ReturnType> Algo;
    typename Algo::ArgsType args(q.derived(),v.derived(),res);
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
    return res;
  }

  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<JointCollection> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    return interpolate<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2,Scalar>(model, q0, q1, u);
  }

  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<JointCollection> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    typedef InterpolateStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar,ReturnType> Algo;
    
    ReturnType res(model.nq);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(q0, q1, u, res));
    }
    
    return res;
  }

  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<JointCollection> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    
    ReturnType res(model.nv);
    
    typedef DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    typename Algo::ArgsType args(q0.derived(),q1.derived(),res);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
    
    return res;
  }

  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<JointCollection> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return difference<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }

  template<typename LieGroup_t,typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<JointCollection> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    
    ReturnType distances(ReturnType::Zero(model.njoints-1));
    typedef SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      typename Algo::ArgsType args(i-1,q0.derived(),q1.derived(), distances);
      Algo::run(model.joints[i], args);
    }
    
    return distances;
  }

  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<JointCollection> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return squaredDistance<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }
  
  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename JointCollection::Scalar
  distance(const ModelTpl<JointCollection> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return std::sqrt(squaredDistance<LieGroup_t,JointCollection,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived()).sum());
  }
  
  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename JointCollection::Scalar
  distance(const ModelTpl<JointCollection> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return std::sqrt(squaredDistance<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived()).sum());
  }

  template<typename LieGroup_t,typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename EIGEN_PLAIN_TYPE(typename ModelTpl<JointCollection>::ConfigVectorType)
  randomConfiguration(const ModelTpl<JointCollection> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename EIGEN_PLAIN_TYPE(typename ModelTpl<JointCollection>::ConfigVectorType) ReturnType;
    
    ReturnType q(model.nq);
    typedef RandomConfigurationStep<LieGroup_t,ReturnType,ConfigVectorIn1,ConfigVectorIn2> Algo;
    typename Algo::ArgsType args(EIGEN_CONST_CAST(ReturnType,q), lowerLimits.derived(), upperLimits.derived());
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
    
    return q;
  }

  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename EIGEN_PLAIN_TYPE(typename ModelTpl<JointCollection>::ConfigVectorType)
  randomConfiguration(const ModelTpl<JointCollection> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    return randomConfiguration<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2>(model, lowerLimits.derived(), upperLimits.derived());
  }

  template<typename LieGroup_t,typename JointCollection>
  typename EIGEN_PLAIN_TYPE(typename ModelTpl<JointCollection>::ConfigVectorType)
  randomConfiguration(const ModelTpl<JointCollection> & model)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::ConfigVectorType ConfigVectorType;
    return randomConfiguration<LieGroup_t,JointCollection,ConfigVectorType,ConfigVectorType>(model, model.lowerPositionLimit, model.upperPositionLimit);
  }

  template<typename JointCollection>
  typename EIGEN_PLAIN_TYPE(typename ModelTpl<JointCollection>::ConfigVectorType)
  randomConfiguration(const ModelTpl<JointCollection> & model)
  {
    return randomConfiguration<LieGroupMap,JointCollection>(model);
  }

  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorType>
  inline void normalize(const ModelTpl<JointCollection> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef NormalizeStep<LieGroup_t,ConfigVectorType> Algo;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(EIGEN_CONST_CAST(ConfigVectorType,qout)));
    }
  }
  
  template<typename JointCollection, typename ConfigVectorType>
  inline void normalize(const ModelTpl<JointCollection> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    return normalize<LieGroupMap,JointCollection,ConfigVectorType>(model,qout.derived());
  }

  template<typename LieGroup_t, typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  inline bool
  isSameConfiguration(const ModelTpl<JointCollection> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec)
  {
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    
    bool result = true;
    typedef IsSameConfigurationStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> Algo;
    typename Algo::ArgsType args(result,q1.derived(),q2.derived(),prec);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
      if(!result)
        return false;
    }
    
    return true;
  }

  template<typename JointCollection, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<JointCollection> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const typename JointCollection::Scalar & prec = Eigen::NumTraits<typename JointCollection::Scalar>::dummy_precision())
  {
    typedef typename JointCollection::Scalar Scalar;
    return isSameConfiguration<LieGroupMap,JointCollection,ConfigVectorIn1,ConfigVectorIn2,Scalar>(model, q1.derived(), q2.derived(), prec);
  }
  
  template<typename LieGroup_t, typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,1,JointCollection::Options>
  neutral(const ModelTpl<JointCollection> & model)
  {
    typedef Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,1,JointCollection::Options> ReturnType;
    typedef ModelTpl<JointCollection> Model;
    typedef typename Model::JointIndex JointIndex;
    
    ReturnType neutral_elt(model.nq);
    
    typename NeutralStep<LieGroup_t,ReturnType>::ArgsType args(neutral_elt.derived());
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i )
    {
      NeutralStep<LieGroup_t,ReturnType>::run(model.joints[i],args);
    }
    
    return neutral_elt;
  }
  
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,1,JointCollection::Options>
  neutral(const ModelTpl<JointCollection> & model)
  {
    return neutral<LieGroupMap,JointCollection>(model);
  }


} // namespace se3

#endif // ifndef __se3_joint_configuration_hxx__

