//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_multibody_pool_geometry_hpp__
#define __pinocchio_multibody_pool_geometry_hpp__

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/pool/model.hpp"

#include "pinocchio/utils/openmp.hpp"

namespace pinocchio
{
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  class GeometryPoolTpl
  : public ModelPoolTpl<_Scalar,_Options,JointCollectionTpl>
  {
  public:
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef ModelPoolTpl<_Scalar,_Options,JointCollectionTpl> Base;
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef typename Base::Model Model;
    typedef typename Base::Data Data;
    typedef typename Base::DataVector DataVector;
    typedef ::pinocchio::GeometryModel GeometryModel;
    typedef ::pinocchio::GeometryData GeometryData;
    
    typedef std::vector<GeometryModel,Eigen::aligned_allocator<GeometryModel> > GeometryModelVector;
    typedef std::vector<GeometryData,Eigen::aligned_allocator<GeometryData> > GeometryDataVector;
    
    /// \brief Default constructor from a model and a pool size.
    ///
    /// \param[in] model input model used for parallel computations.
    /// \param[in] geometry_model input geometry model used for parallel computations.
    /// \param[in] pool_size total size of the pool.
    ///
    GeometryPoolTpl(const Model & model, const GeometryModel & geometry_model,
                    const int pool_size = omp_get_max_threads())
    : Base(model,pool_size)
    , m_geometry_model(geometry_model)
    , m_geometry_datas((size_t)pool_size,GeometryData(geometry_model))
    {}
    
    /// \brief Copy constructor from an other GeometryPoolTpl.
    ///
    /// \param[in] other GeometryPoolTpl to copy.
    ///
    GeometryPoolTpl(const GeometryPoolTpl & other)
    : Base(other)
    , m_geometry_model(other.m_geometry_model)
    , m_geometry_datas(other.m_geometry_datas)
    {}
    
    /// \brief Returns the geometry model
    const GeometryModel & geometry_model() const { return m_geometry_model; }
    
    /// \brief Returns the geometry model
    GeometryModel & geometry_model() { return m_geometry_model; }
    
    /// \brief Returns the geometry_data at index
    const GeometryData & geometry_data(const size_t index) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_geometry_datas.size(),
                                     "Index greater than the size of the geometry_datas vector.");
      return m_geometry_datas[index];
    }
    
    /// \brief Returns the geometry_data at index
    GeometryData & geometry_data(const size_t index)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_geometry_datas.size(),
                                     "Index greater than the size of the geometry_datas vector.");
      return m_geometry_datas[index];
    }
    
    /// \brief Vector of Geometry Data
    const GeometryDataVector & geometry_datas() const { return m_geometry_datas; }
    
    /// \brief Vector of Geometry Data
    GeometryDataVector & geometry_datas() { return m_geometry_datas; }
    
    using Base::update;
    using Base::size;
    using Base::model;
    using Base::datas;
    
    ///
    /// \brief Update the geometry datas with the new value
    ///
    /// \param[in] geometry_data new geometry data value
    ///
    void update(const GeometryData & geometry_data)
    {
      std::fill(m_geometry_datas.begin(),m_geometry_datas.end(),geometry_data);
    }
    
    ///
    /// \brief Update the geometry model with the new input value.
    ///        In this case, all the geometry_datas will be replaced
    ///
    /// \param[in] geometry_model new geometry model value.
    ///
    void update(const GeometryModel & geometry_model)
    {
      m_geometry_model = geometry_model;
      std::fill(m_geometry_datas.begin(),m_geometry_datas.end(),
                GeometryData(m_geometry_model));
    }
    
    ///
    /// \brief Update the geometry model and data with the new input values.
    ///        In this case, all the geometry_datas will be replaced
    ///
    /// \param[in] geometry_model new geometry model value.
    /// \param[in] geometry_data new geometry data value
    ///
    void update(const GeometryModel & geometry_model,
                const GeometryData & geometry_data)
    {
      m_geometry_model = geometry_model;
      update(geometry_data);
    }
    
    /// \brief Destructor
    virtual ~GeometryPoolTpl() {};
    
  protected:
    
    /// \brief Geometry Model associated to the pool.
    GeometryModel m_geometry_model;
    
    /// \brief Vector of Geometry Data associated to the pool.
    GeometryDataVector m_geometry_datas;
      
    /// \brief Method to implement in the derived classes.
    virtual void do_resize(const int new_size)
    {
      m_geometry_datas.resize((size_t)new_size);
      if(size() < new_size)
      {
        typename GeometryDataVector::iterator it = m_geometry_datas.begin();
        std::advance(it, (long)(new_size - size()));
        std::fill(it,m_geometry_datas.end(),m_geometry_datas[0]);
      }
    }
    
  };

  typedef GeometryPoolTpl<double,0,JointCollectionDefaultTpl> GeometryPool;
}

#endif // ifndef __pinocchio_multibody_pool_geometry_hpp__
