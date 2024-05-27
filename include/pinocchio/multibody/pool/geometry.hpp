//
// Copyright (c) 2021-2022 INRIA
//

#ifndef __pinocchio_multibody_pool_geometry_hpp__
#define __pinocchio_multibody_pool_geometry_hpp__

#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/pool/model.hpp"

namespace pinocchio
{
  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  class GeometryPoolTpl : public ModelPoolTpl<_Scalar, _Options, JointCollectionTpl>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ModelPoolTpl<_Scalar, _Options, JointCollectionTpl> Base;
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef typename Base::Model Model;
    typedef typename Base::Data Data;
    typedef typename Base::ModelVector ModelVector;
    typedef typename Base::DataVector DataVector;
    typedef ::pinocchio::GeometryModel GeometryModel;
    typedef ::pinocchio::GeometryData GeometryData;

    typedef std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> GeometryModelVector;
    typedef std::vector<GeometryData, Eigen::aligned_allocator<GeometryData>> GeometryDataVector;

    /// \brief Default constructor from a model and a pool size.
    ///
    /// \param[in] model input model used for parallel computations.
    /// \param[in] geometry_model input geometry model used for parallel computations.
    /// \param[in] pool_size total size of the pool.
    ///
    GeometryPoolTpl(
      const Model & model,
      const GeometryModel & geometry_model,
      const size_t pool_size = (size_t)omp_get_max_threads())
    : Base(model, pool_size)
    {
      m_geometry_models.reserve(pool_size);
      m_geometry_datas.reserve(pool_size);
      for (size_t k = 0; k < pool_size; ++k)
      {
        m_geometry_models.push_back(geometry_model.clone());
        m_geometry_datas.push_back(GeometryData(m_geometry_models[k]));

        typedef typename GeometryData::SE3 SE3;
        for (SE3 & oMg_i : m_geometry_datas.back().oMg)
        {
          oMg_i.setIdentity();
        }
      }
    }

    /// \brief Copy constructor from an other GeometryPoolTpl.
    ///
    /// \param[in] other GeometryPoolTpl to copy.
    ///
    GeometryPoolTpl(const GeometryPoolTpl & other)
    : Base(other)
    {
      const size_t pool_size = other.size();
      m_geometry_models.reserve(pool_size);
      m_geometry_datas.reserve(pool_size);
      for (size_t k = 0; k < pool_size; ++k)
      {
        m_geometry_models.push_back(other.m_geometry_models[k].clone());
        m_geometry_datas.push_back(GeometryData(m_geometry_models[k]));
      }
    }

    /// \brief Returns the geometry_model at given index
    const GeometryModel & getGeometryModel(const size_t index) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        index < m_geometry_models.size(),
        "Index greater than the size of the geometry_models vector.");
      return m_geometry_models[index];
    }

    /// \brief Returns the geometry_model at given index
    GeometryModel & getGeometryModel(const size_t index)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        index < m_geometry_models.size(),
        "Index greater than the size of the geometry_models vector.");
      return m_geometry_models[index];
    }

    /// \brief Returns the geometry_data at given index
    const GeometryData & getGeometryData(const size_t index) const
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        index < m_geometry_datas.size(),
        "Index greater than the size of the geometry_datas vector.");
      return m_geometry_datas[index];
    }

    /// \brief Returns the geometry_data at given index
    GeometryData & getGeometryData(const size_t index)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        index < m_geometry_datas.size(),
        "Index greater than the size of the geometry_datas vector.");
      return m_geometry_datas[index];
    }

    /// \brief Returns the vector of Geometry Data
    const GeometryDataVector & getGeometryDatas() const
    {
      return m_geometry_datas;
    }

    /// \brief Returns the vector of Geometry Data
    GeometryDataVector & getGeometryDatas()
    {
      return m_geometry_datas;
    }

    /// \brief Returns the vector of Geometry Model
    const GeometryModelVector & getGeometryModels() const
    {
      return m_geometry_models;
    }

    /// \brief Returns the vector of Geometry Model
    GeometryModelVector & getGeometryModels()
    {
      return m_geometry_models;
    }

    using Base::size;
    using Base::update;

    ///  \brief Synchronize the internal geometry models with the input geometry for all given
    /// geometry indexes by cloning the related geometryObjects.
    void sync(const GeometryModel & geometry_model, const std::vector<GeomIndex> & geometry_indexes)
    {
      for (GeomIndex i : geometry_indexes)
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
          i < geometry_model.ngeoms,
          "One of the given geometry index is greater than geometry_model.ngeoms.");

      for (GeometryModel & geometry_model_pool : m_geometry_models)
      {
        for (GeomIndex i : geometry_indexes)
          geometry_model_pool.geometryObjects[i] = geometry_model.geometryObjects[i].clone();
      }
    }

    ///
    ///  \brief Update the geometry datas with the new value
    ///
    /// \param[in] geometry_data_to_copy new geometry data value to copy
    ///
    virtual void update(const GeometryData & geometry_data_to_copy)
    {
      for (GeometryData & geometry_data : m_geometry_datas)
      {
        geometry_data.oMg = geometry_data_to_copy.oMg;
        geometry_data.activeCollisionPairs = geometry_data_to_copy.activeCollisionPairs;
        geometry_data.distanceRequests = geometry_data_to_copy.distanceRequests;
        geometry_data.collisionRequests = geometry_data_to_copy.collisionRequests;
        geometry_data.collisionPairIndex = geometry_data_to_copy.collisionPairIndex;
      }
    }

    ///  \brief Destructor
    virtual ~GeometryPoolTpl() {};

  protected:
    /// \brief Vector of Geometry Model associated to the pool.
    GeometryModelVector m_geometry_models;

    /// \brief Vector of Geometry Data associated to the pool.
    GeometryDataVector m_geometry_datas;

    ///  \brief Method to implement in the derived classes.
    virtual void doResize(const size_t new_size)
    {
      const size_t current_size = (size_t)size();
      m_geometry_models.resize((size_t)new_size);
      m_geometry_datas.resize((size_t)new_size);
      if (current_size < new_size)
      {
        for (size_t k = current_size; k < new_size; ++k)
        {
          m_geometry_models[k] = m_geometry_models[0].clone();
          m_geometry_datas[k] = GeometryData(m_geometry_models[k]);
        }
      }
    }
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_pool_geometry_hpp__
