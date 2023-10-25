//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_parallel_aba_hpp__
#define __pinocchio_algorithm_parallel_aba_hpp__

#include <omp.h>

#include "pinocchio/multibody/pool/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
  
namespace pinocchio
{
  ///
  /// \brief A parallel version of the Articulated Body algorithm. It computes the forward dynamics, aka the joint acceleration according to the current state of the system and the desired joint torque.
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorPool Matrix type of the joint configuration vector.
  /// \tparam TangentVectorPool1 Matrix type of the joint velocity vector.
  /// \tparam TangentVectorPool2 Matrix type of the joint torque vector.
  /// \tparam TangentVectorPool3 Matrix type of the joint acceleration vector.
  ///
  /// \param[in] pool Pool containing model and data for parallel computations.
  /// \param[in] num_threads Number of threads used for parallel computations.
  /// \param[in] q The joint configuration vector (dim model.nq x batch_size).
  /// \param[in] v The joint velocity vector (dim model.nv x batch_size).
  /// \param[in] tau The joint acceleration vector (dim model.nv x batch_size).
  /// \param[out] a The joint torque vector (dim model.nv x batch_size).
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorPool, typename TangentVectorPool1, typename TangentVectorPool2, typename TangentVectorPool3>
  void aba(const int num_threads,
           ModelPoolTpl<Scalar,Options,JointCollectionTpl> & pool,
           const Eigen::MatrixBase<ConfigVectorPool> & q,
           const Eigen::MatrixBase<TangentVectorPool1> & v,
           const Eigen::MatrixBase<TangentVectorPool2> & tau,
           const Eigen::MatrixBase<TangentVectorPool3> & a)
  {
    typedef ModelPoolTpl<Scalar,Options,JointCollectionTpl> Pool;
    typedef typename Pool::Model Model;
    typedef typename Pool::Data Data;
    typedef typename Pool::DataVector DataVector;
    
    const Model & model = pool.model();
    DataVector & datas = pool.datas();
    TangentVectorPool3 & res = a.const_cast_derived();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(num_threads <= pool.size(), "The pool is too small");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.rows(), model.nq);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(v.rows(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(a.rows(), model.nv);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), model.nv);
    
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), v.cols());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), a.cols());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(q.cols(), res.cols());
    
    omp_set_num_threads(num_threads);
    const Eigen::DenseIndex batch_size = res.cols();
    Eigen::DenseIndex i = 0;
    
#pragma omp parallel for
    for(i = 0; i < batch_size; i++)
    {
      const int thread_id = omp_get_thread_num();
      Data & data = datas[(size_t)thread_id];
      res.col(i) = aba(model,data,q.col(i),v.col(i),tau.col(i));
    }
  }
}

#endif // ifndef __pinocchio_algorithm_parallel_aba_hpp__
