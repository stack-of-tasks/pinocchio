//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_joint_configuration_hxx__
#define __pinocchio_joint_configuration_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <cmath>

/* --- Details -------------------------------------------------------------------- */
namespace pinocchio
{

  // --------------- API with return value as argument ---------------------- //

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ReturnType>
  void
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v,
            const Eigen::MatrixBase<ReturnType> & qout)
  {
    assert(q.size() == model.nq && "The configuration vector is not of the right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of the right size");
    assert(qout.size() == model.nq && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & res = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, qout);

    typedef IntegrateStep<LieGroup_t,ConfigVectorType,TangentVectorType,ReturnType> Algo;
    typename Algo::ArgsType args(q.derived(),v.derived(),res);
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u,
              const Eigen::MatrixBase<ReturnType> & qout)
  {
    assert(q0.size() == model.nq && "The first configuration vector is not of the right size");
    assert(q1.size() == model.nq && "The second configuration vector is not of the right size");
    assert(qout.size() == model.nq && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & res = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, qout);

    typedef InterpolateStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar,ReturnType> Algo;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(q0.derived(), q1.derived(), u, res.derived()));
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1,
             const Eigen::MatrixBase<ReturnType> & dvout)
  {
    assert(q0.size() == model.nq && "The first configuration vector is not of the right size");
    assert(q1.size() == model.nq && "The second configuration vector is not of the right size");
    assert(dvout.size() == model.nv && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & res = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, dvout);

    typedef DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    typename Algo::ArgsType args(q0.derived(),q1.derived(),res);
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                  const Eigen::MatrixBase<ReturnType> & out)
  {
    assert(q0.size() == model.nq && "The first configuration vector is not of the right size");
    assert(q1.size() == model.nq && "The second configuration vector is not of the right size");
    assert(out.size() == (model.njoints-1) && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & distances = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, out);

    typedef SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,ReturnType> Algo;
    for(JointIndex i=0; i<(JointIndex) model.njoints-1; ++i)
    {
      typename Algo::ArgsType args(i,q0.derived(),q1.derived(), distances.derived());
      Algo::run(model.joints[i+1], args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2, typename ReturnType>
  void
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits,
                      const Eigen::MatrixBase<ReturnType> & qout)
  {
    assert(lowerLimits.size() == model.nq && "The lower limits vector is not of the right size");
    assert(upperLimits.size() == model.nq && "The upper limits vector is not of the right size");
    assert(qout.size() == model.nq && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & q = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, qout);

    typedef RandomConfigurationStep<LieGroup_t,ReturnType,ConfigVectorIn1,ConfigVectorIn2> Algo;
    typename Algo::ArgsType args(PINOCCHIO_EIGEN_CONST_CAST(ReturnType,q), lowerLimits.derived(), upperLimits.derived());
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ReturnType>
  void
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model, const Eigen::MatrixBase<ReturnType> & qout)
  {
    assert(qout.size() == model.nq && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    ReturnType & neutral_elt = PINOCCHIO_EIGEN_CONST_CAST(ReturnType, qout);
    
    typename NeutralStep<LieGroup_t,ReturnType>::ArgsType args(neutral_elt.derived());
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i )
    {
      NeutralStep<LieGroup_t,ReturnType>::run(model.joints[i],args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename JacobianMatrixType>
  void dIntegrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v,
                  const Eigen::MatrixBase<JacobianMatrixType> & J,
                  const ArgumentPosition arg)
  {
    assert(q.size() == model.nq && "The configuration vector is not of the right size");
    assert(v.size() == model.nv && "The joint velocity vector is not of the right size");
    assert(J.rows() == model.nv && "The output argument is not of the right size");
    assert(J.cols() == model.nv && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef dIntegrateStep<LieGroup_t,ConfigVectorType,TangentVectorType,JacobianMatrixType> Algo;
    typename Algo::ArgsType args(q.derived(),v.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,J),arg);
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i], args);
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar
  squaredDistanceSum(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                     const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                     const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    assert(q0.size() == model.nq && "The first configuration vector is not of the right size");
    assert(q1.size() == model.nq && "The second configuration vector is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typename ConfigVectorIn1::Scalar squaredDistance = 0.0;

    typedef SquaredDistanceSumStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> Algo;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      typename Algo::ArgsType args(q0.derived(),q1.derived(), squaredDistance);
      Algo::run(model.joints[i], args);
    }
    
    return squaredDistance;
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  Scalar
  distance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
           const Eigen::MatrixBase<ConfigVectorIn1> & q0,
           const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    const Scalar & squaredDistance = squaredDistanceSum<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2>(model, q0.derived(), q1.derived());
    return math::sqrt(squaredDistance);
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline void normalize(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const Eigen::MatrixBase<ConfigVectorType> & qout)
  {
    assert(qout.size() == model.nq && "The output argument is not of the right size");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    
    typedef NormalizeStep<LieGroup_t,ConfigVectorType> Algo;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],
                typename Algo::ArgsType(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType,qout)));
    }
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline bool
  isSameConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                      const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                      const Scalar & prec)
  {
    assert(q1.size() == model.nq && "The first configuration vector is not of the right size");
    assert(q2.size() == model.nq && "The second configuration vector is not of the right size");
    assert(prec >= 0 && "The precision is negative");

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

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVector, typename JacobianMatrix>
  inline void
  integrateCoeffWiseJacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const Eigen::MatrixBase<ConfigVector> & q,
                             const Eigen::MatrixBase<JacobianMatrix> & jacobian)
  {
    assert(q.size() == model.nq && "The configuration vector is not of the right size");
    assert(jacobian.rows() == model.nq && jacobian.cols() == model.nv
           && "The jacobian does not have the right dimension");

    typedef IntegrateCoeffWiseJacobianStep<LieGroup_t,ConfigVector,JacobianMatrix> Algo;
    typename Algo::ArgsType args(q.derived(),PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,jacobian));
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Algo::run(model.joints[i],args);
    }
  }

  // ----------------- API that allocates memory ---------------------------- //

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType)
  integrate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
            const Eigen::MatrixBase<ConfigVectorType> & q,
            const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorType) ReturnType;
    ReturnType res(model.nq);
    integrate<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType,ReturnType>(model, q.derived(), v.derived(), res);
    return res;
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  interpolate(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const Eigen::MatrixBase<ConfigVectorIn1> & q0,
              const Eigen::MatrixBase<ConfigVectorIn2> & q1,
              const Scalar & u)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    ReturnType res(model.nq);
    interpolate<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, q0, q1, u, res);
    return res;
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  difference(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const Eigen::MatrixBase<ConfigVectorIn1> & q0,
             const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    ReturnType res(model.nv);
    difference<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),res);
    return res;
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  inline typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1)
  squaredDistance(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                  const Eigen::MatrixBase<ConfigVectorIn2> & q1)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(ConfigVectorIn1) ReturnType;
    ReturnType distances(ReturnType::Zero(model.njoints-1));
    squaredDistance<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model,q0.derived(),q1.derived(),distances);    
    return distances;
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorIn1, typename ConfigVectorIn2>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                      const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType)) ReturnType; 
    ReturnType q(model.nq);
    randomConfiguration<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorIn1,ConfigVectorIn2,ReturnType>(model, lowerLimits.derived(), upperLimits.derived(), q);
    return q;
  }

  template<typename LieGroup_t,typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  typename PINOCCHIO_EIGEN_PLAIN_TYPE((typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType))
  randomConfiguration(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::ConfigVectorType ConfigVectorType;
    return randomConfiguration<LieGroup_t,Scalar,Options,JointCollectionTpl,ConfigVectorType,ConfigVectorType>(model, model.lowerPositionLimit, model.upperPositionLimit);
  }

  template<typename LieGroup_t, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options>
  neutral(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ReturnType;    
    ReturnType q(model.nq);
    neutral<LieGroup_t,Scalar,Options,JointCollectionTpl,ReturnType>(model,q);
    return q;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_joint_configuration_hxx__

