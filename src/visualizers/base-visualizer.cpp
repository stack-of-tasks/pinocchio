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
      const GeometryModel * collision_model)
    : m_model(&model)
    , m_visualModel(&visual_model)
    , m_collisionModel(collision_model)
    , data(model)
    , visualData(visual_model)
    , collisionData()
    {
      if (m_collisionModel)
        collisionData = GeometryData(*m_collisionModel);
    }

    void BaseVisualizer::rebuildData()
    {
      data = Data(*m_model);
      visualData = GeometryData(*m_visualModel);
      if (m_collisionModel)
        collisionData = GeometryData(*m_collisionModel);
    }

    void BaseVisualizer::display(const boost::optional<ConstVectorRef> & q)
    {
      displayPrecall();
      if (q.has_value())
      {
        forwardKinematics(*m_model, data, *q);
      }
      updateGeometryPlacements(*m_model, data, *m_visualModel, visualData);
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
