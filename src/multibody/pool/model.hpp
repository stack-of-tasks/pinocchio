//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_pool_model_hpp__
#define __pinocchio_multibody_pool_model_hpp__

#include <algorithm>
#include <omp.h>

#include "pinocchio/multibody/pool/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/utils/openmp.hpp"

namespace pinocchio
{
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  class ModelPoolTpl
  {
  public:
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef std::vector<Model,Eigen::aligned_allocator<Model> > ModelVector;
    typedef std::vector<Data,Eigen::aligned_allocator<Data> > DataVector;
    
    /// \brief Default constructor from a model and a pool size.
    ///
    /// \param[in] model_ptr input model pointer used for parallel computations.
    /// \param[in] pool_size total size of the pool.
    ///
    explicit ModelPoolTpl(const Model * model_ptr,
                          const size_t pool_size = (size_t)omp_get_max_threads())
    : m_model_ptr(model_ptr)
    , m_datas(pool_size, Data(*model_ptr))
    {}
    
    /// \brief Copy constructor from an other PoolModel.
    ///
    /// \param[in] pool_model PoolModel to copy.
    ///
    ModelPoolTpl(const ModelPoolTpl & pool_model)
    : m_model_ptr(pool_model.m_model_ptr)
    , m_datas(pool_model.m_datas)
    {}
    
    /// \brief Returns the model stored within the pool.
    const Model & getModel() const { return *m_model_ptr; }
    
    /// \brief Update all the datas with the input data value.
    ///
    /// \param[in] data new value to use and to copy within the vector of data.
    ///
    void update(const Data & data)
    {
      std::fill(m_datas.begin(),m_datas.end(),data);
    }
    
    /// \brief Returns the size of the pool.
    size_t size() const { return m_datas.size(); }
    
    /// \brief Set the size of the pool and perform the appropriate resize.
    void resize(const size_t new_size)
    {
      const size_t size = m_datas.size();
      m_datas.resize((size_t)new_size);
      if(size < new_size)
      {
        typename DataVector::iterator it = m_datas.begin();
        std::advance(it, (long)(new_size - size));
        std::fill(it,m_datas.end(),m_datas[0]);
      }
      doResize(new_size); // call Derived::doResize();
    }

    /// \brief Returns the data vectors
    const DataVector & getDatas() const { return m_datas; }
    
    /// \brief Returns the data vectors
    DataVector & getDatas() { return m_datas; }
    
    /// \brief Return a specific data
    const Data & getData(const size_t index) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_datas.size(),
                                     "Index greater than the size of the datas vector.");
      return m_datas[index];
    }
    
    /// \brief Returns a specific data
    Data & getData(const size_t index)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_datas.size(),
                                     "Index greater than the size of the datas vector.");
      return m_datas[index];
    }
    
    /// \brief Destructor
    virtual ~ModelPoolTpl() {};
    
  protected:
    
    /// \brief Model stored within the pool.
    const Model * m_model_ptr;
    
    /// \brief Vector of data elements
    DataVector m_datas;
    
    /// \brief Method to implement in the derived classes.
    virtual void doResize(const size_t new_size)
    {
      PINOCCHIO_UNUSED_VARIABLE(new_size);
    }
    
  };

}

#endif // ifndef __pinocchio_multibody_pool_model_hpp__
