//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_multibody_pool_model_hpp__
#define __pinocchio_multibody_pool_model_hpp__

#include <algorithm>
#include <omp.h>

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
    /// \param[in] model input model used for parallel computations.
    /// \param[in] pool_size total size of the pool.
    ///
    explicit ModelPoolTpl(const Model & model,
                          const int pool_size = omp_get_max_threads())
    : m_model(model)
    , m_datas((size_t)pool_size, Data(model))
    , m_size(pool_size)
    {}
    
    /// \brief Copy constructor from an other PoolModel.
    ///
    /// \param[in] pool_model PoolModel to copy.
    ///
    ModelPoolTpl(const ModelPoolTpl & pool_model)
    : m_model(pool_model.m_model)
    , m_datas(pool_model.m_datas)
    , m_size(pool_model.m_size)
    {}
    
    /// \brief Returns the model stored within the pool.
    const Model & model() const { return m_model; }
    
    /// \brief Returns the model stored within the pool.
    Model & model() { return m_model; }
    
    /// \brief Update the model, meaning that all the datas will be refreshed accordingly.
    ///
    /// \param[in] model new model value.
    ///
    void update(const Model & model)
    {
      m_model = model;
      update(Data(model));
    }
    
    /// \brief Update all the datas with the input data value.
    ///
    /// \param[in] data new value to use and to copy within the vector of data.
    ///
    void update(const Data & data)
    {
      std::fill(m_datas.begin(),m_datas.end(),data);
    }

    ///
    /// \brief Update the model and data with the new input values.
    ///        In this case, all the geometry_datas will be replaced
    ///
    /// \param[in] geometry_model new geometry model value.
    /// \param[in] geometry_data new geometry data value
    ///
    void update(const Model & model,
                const Data & data)
    {
      m_model = model;
      update(data);
    }
    
    /// \brief Returns the size of the pool.
    int size() const { return m_size; }
    
    /// \brief Set the size of the pool and perform the appropriate resize.
    void resize(const int new_size)
    {
      m_datas.resize((size_t)new_size);
      if(m_size < new_size)
      {
        typename DataVector::iterator it = m_datas.begin();
        std::advance(it, (long)(new_size - m_size));
        std::fill(it,m_datas.end(),m_datas[0]);
      }
      do_resize(new_size); // call Derived::do_resize();
      m_size = new_size;
    }

    /// \brief Returns the data vectors
    const DataVector & datas() const { return m_datas; }
    
    /// \brief Returns the data vectors
    DataVector & datas() { return m_datas; }
    
    /// \brief Return a specific data
    const Data & data(const size_t index) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_datas.size(),
                                     "Index greater than the size of the datas vector.");
      return m_datas[index];
    }
    
    /// \brief Returns a specific data
    Data & data(const size_t index)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_datas.size(),
                                     "Index greater than the size of the datas vector.");
      return m_datas[index];
    }
    
    /// \brief Destructor
    virtual ~ModelPoolTpl() {};
    
  protected:
    
    /// \brief Model stored within the pool.
    Model m_model;
    
    /// \brief Vector of data elements
    DataVector m_datas;
    
    /// \brief Number of threads used for parallel computations
    int m_size;
    
    /// \brief Method to implement in the derived classes.
    virtual void do_resize(const int new_size)
    {
      PINOCCHIO_UNUSED_VARIABLE(new_size);
    }
    
  };

  typedef ModelPoolTpl<double,0,JointCollectionDefaultTpl> ModelPool;
}

#endif // ifndef __pinocchio_multibody_pool_model_hpp__
