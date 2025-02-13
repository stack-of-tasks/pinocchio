//
// Copyright (c) 2024-2025 INRIA
//
#include "pinocchio/visualizers/base-visualizer.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <chrono>
#include <thread>

namespace pinocchio
{
  namespace visualizers
  {

    BaseVisualizer::BaseVisualizer(
      const Model & model,
      const GeometryModel & visual_model,
      const GeometryModel * collision_model,
      Data & data,
      GeometryData & visual_data,
      GeometryData * collision_data)
    : m_model(model)
    , m_visualModel(&visual_model)
    , m_collisionModel(collision_model)
    , m_data(&data)
    , m_visualData(&visual_data)
    , m_collisionData(collision_data)
    , m_ownedData(false)
    {
      if (hasCollisionModel())
      {
        PINOCCHIO_THROW(
          collision_data != nullptr, std::logic_error,
          "A collision model was provided but no pointer to collision GeometryData to borrow.");
      }
      else
      {
        m_collisionData = nullptr;
      }
    }

    BaseVisualizer::BaseVisualizer(
      const Model & model,
      const GeometryModel & visual_model,
      const GeometryModel * collision_model)
    : m_model(model)
    , m_visualModel(&visual_model)
    , m_collisionModel(collision_model)
    , m_data(new Data(model))
    , m_visualData(new GeometryData(visual_model))
    , m_ownedData(true)
    {
      if (hasCollisionModel())
        m_collisionData = new GeometryData(*m_collisionModel);
    }

    BaseVisualizer::~BaseVisualizer()
    {
      this->destroyData();
    }

    void BaseVisualizer::destroyData()
    {
      if (m_ownedData)
      {
        assert(m_data);
        assert(m_visualData);
        delete m_data;
        delete m_visualData;
        if (m_collisionData)
          delete m_collisionData;
      }
      m_data = nullptr;
      m_visualData = nullptr;
      m_collisionData = nullptr;
    }

    void BaseVisualizer::rebuildData()
    {
      this->destroyData();
      m_data = new Data(m_model);
      m_visualData = new GeometryData(*m_visualModel);
      if (hasCollisionModel())
        m_collisionData = new GeometryData(*m_collisionModel);
      // mark as owned, if that was not the case
      m_ownedData = true;
    }

    void BaseVisualizer::display(const boost::optional<ConstVectorRef> & q)
    {
      displayPrecall();
      if (q.has_value())
      {
        forwardKinematics(model(), *m_data, *q);
      }
      updateGeometryPlacements(model(), *m_data, *m_visualModel, *m_visualData);
      if (hasCollisionModel())
      {
        updateGeometryPlacements(model(), *m_data, *m_collisionModel, *m_collisionData);
      }
      displayImpl();
    }

    void BaseVisualizer::play(const std::vector<ConstVectorRef> & qs, Scalar dt)
    {
      using std::chrono::steady_clock;
      const auto nsteps = qs.size();
      const auto ms = std::chrono::milliseconds(unsigned(dt * 1e3));

      for (size_t i = 0; i < nsteps; i++)
      {
        const auto cur = steady_clock::now();
        this->display(qs[i]);
        if (!this->forceRedraw())
          return;
        std::this_thread::sleep_until(cur + ms);
      }
    }

    void BaseVisualizer::play(const ConstMatrixRef & qs, Scalar dt)
    {
      using Eigen::Index;
      const Index nsteps = qs.rows();
      std::vector<ConstVectorRef> qs_;
      for (Index i = 0; i < nsteps; i++)
      {
        qs_.emplace_back(qs.row(i));
      }
      // call overload
      this->play(qs_, dt);
    }

  } // namespace visualizers
} // namespace pinocchio
