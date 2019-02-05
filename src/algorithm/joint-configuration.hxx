//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_joint_configuration_hxx__
#define __pinocchio_joint_configuration_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <cmath>

/* --- Details -------------------------------------------------------------------- */
namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    return integrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType>(model, q.derived(), v.derived());
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType) ReturnType;
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
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
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg)
  {
    return dIntegrate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,JacobianMatrixType>(model, q.derived(), v.derived(), J.derived(),arg);
  }
  
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef dIntegrateStep<LieGroup_t,ConfigVectorType,TangentVectorType,JacobianMatrixType> Algo;
    typename Algo::ArgsType args(q.derived(),v.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,J),arg);
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    return interpolate<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0, q1, u);
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    typedef InterpolateStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar,ReturnType> Algo;
    
    ReturnType res(model.nq);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(q0, q1, u, res));
    }
    
    return res;
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    
    ReturnType res(model.nv);
    
    typedef DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    typename Algo::ArgsType args(q0.derived(),q1.derived(),res);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
    
    return res;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return difference<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    
    ReturnType distances(ReturnType::Zero(model.njoints-1));
    typedef SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    for(JointIndex i=0; i<(JointIndex) model.njoints-1; ++i)
    {
      typename Algo::ArgsType args(i,q0.derived(),q1.derived(), distances);
      Algo::run(model.joints[i+1], args);
    }
    
    return distances;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return squaredDistance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model,q0.derived(),q1.derived());
  }
  
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar
  distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return math::sqrt(squaredDistance<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived()).sum());
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline Scalar
  distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    return math::sqrt(squaredDistance<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived()).sum());
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType)) ReturnType;
    
    ReturnType q(model.nq);
    typedef RandomConfigurationStep<LieGroup_t,ReturnType,ConfigVectorIn1,ConfigVectorIn2> Algo;
    typename Algo::ArgsType args(PINOCCHIO_EIGEN_CONST_CAST(ReturnType,q), lowerLimits.derived(), upperLimits.derived());
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
    
    return q;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, lowerLimits.derived(), upperLimits.derived());
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::ConfigVectorType ConfigVectorType;
    return randomConfiguration<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorType,ConfigVectorType>(model, model.lowerPositionLimit, model.upperPositionLimit);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    return randomConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl>(model);
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef NormalizeStep<LieGroup_t,ConfigVectorType> Algo;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType,qout)));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    return normalize<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorType>(model,qout.derived());
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
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

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision())
  {
    return isSameConfiguration<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q1.derived(), q2.derived(), prec);
  }
  
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ReturnType;
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    ReturnType neutral_elt(model.nq);
    
    typename NeutralStep<LieGroup_t,ReturnType>::ArgsType args(neutral_elt.derived());
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i )
    {
      NeutralStep<LieGroup_t,ReturnType>::run(model.joints[i],args);
    }
    
    return neutral_elt;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    return neutral<LieGroupMap,Scalar,Options,JointCollectionTpl>(model);
  }
  
  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector, typename JacobianMatrix>
  inline void
  integrateCoeffWiseJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const Eigen::MatrixBase<ConfigVector> & q,
                             const Eigen::MatrixBase<JacobianMatrix> & jacobian)
  {
    assert(jacobian.rows() == model.nq && jacobian.cols() == model.nv
           && "The jacobian does not have the right dimension");
    
    typedef IntegrateCoeffWiseJacobianStep<LieGroup_t,ConfigVector,JacobianMatrix> Algo;
    typename Algo::ArgsType args(q.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,jacobian));
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],args);
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector, typename JacobianMatrix>
  inline void
  integrateCoeffWiseJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const Eigen::MatrixBase<ConfigVector> & q,
                             const Eigen::MatrixBase<JacobianMatrix> & jacobian)
  {
    return integrateCoeffWiseJacobian<LieGroupMap,Scalar,Options,JointCollectionTpl,ConfigVector,JacobianMatrix>(model,q,jacobian);
  }


} // namespace pinocchio

#endif // ifndef __pinocchio_joint_configuration_hxx__

