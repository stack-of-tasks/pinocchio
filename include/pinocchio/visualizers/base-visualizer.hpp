//
// Copyright (c) 2024-2025 INRIA
//

#ifndef __pinocchio_extra_base_visualizer_hpp__
#define __pinocchio_extra_base_visualizer_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include <boost/optional.hpp>
#include <utility>

namespace pinocchio
{
  namespace visualizers
  {
    using context::Data;
    using context::MatrixXs;
    using context::Model;
    using context::Options;
    using context::Scalar;
    using context::Vector3;
    using context::VectorXs;
    using ::pinocchio::SE3;

    typedef Eigen::Ref<const VectorXs> ConstVectorRef;
    typedef Eigen::Ref<const MatrixXs> ConstMatrixRef;

    /// @brief A base class for defining visualizers for Pinocchio in C++. This
    /// provides basic building blocks (a base constructor, data members, getters
    /// for the models).
    /// @details The base API assumes that the visualizer is not the owner of the
    /// underlying Pinocchio multibody, visual or collision models. Their lifetimes
    /// should be managed by the application context itself.
    /// @remark C++ port of the %BaseVisualizer abstract class in Pinocchio's Python
    /// bindings.
    class BaseVisualizer
    {
    public:
      typedef SE3::Matrix4 Matrix4;

      BaseVisualizer(
        const Model & model,
        const GeometryModel & visual_model,
        const GeometryModel * collision_model = nullptr);

      /// @brief Initialize the viewer.
      virtual void initViewer()
      {
      }

      /// @brief Load the Pinocchio model.
      virtual void loadViewerModel() = 0;

      /// @brief Re-build data objects. Required if the models were modified.
      virtual void rebuildData();

      /// @brief Display configuration @p q (if an actual value is given) or update
      /// the Pinocchio frames.
      virtual void display(const boost::optional<ConstVectorRef> & q = boost::none);

      /// @brief Play an entire trajectory, waiting for time @p dt between each
      /// keyframe.
      virtual void play(const std::vector<ConstVectorRef> & qs, Scalar dt);

      void play(const ConstMatrixRef & qs, Scalar dt);

      /// @brief Override this in child class when the scene has to be redrawn.
      /// Useful for @ref play().
      virtual bool forceRedraw()
      {
        return true;
      }

      /// @brief Set the active camera target.
      virtual void setCameraTarget(const Eigen::Ref<const Vector3> & /*target*/)
      {
      }

      /// @brief Set the active camera position.
      virtual void setCameraPosition(const Eigen::Ref<const Vector3> & /*position*/)
      {
      }

      /// @brief Set the active camera 6D pose.
      virtual void setCameraPose(const Eigen::Ref<const Matrix4> & /*pose*/)
      {
      }

      /// @copybrief setCameraPose()
      inline void setCameraPose(const SE3 & pose)
      {
        this->setCameraPose(pose.toHomogeneousMatrix());
      }

      /// @brief Set camera zoom level; what this means depends on the
      /// implementation (FOV zoom or moving forwards).
      virtual void setCameraZoom(Scalar /*value*/)
      {
      }

      /// @brief Enable/disable controlling the camera from keyboard and mouse.
      virtual void enableCameraControl(bool)
      {
      }

      /// @brief Delete all objects from the scene.
      virtual void clean()
      {
      }

      virtual ~BaseVisualizer() = default;

      const Model & model() const
      {
        return m_model;
      }

      const GeometryModel & visualModel() const
      {
        return *m_visualModel;
      }

      const GeometryModel & collisionModel() const
      {
        PINOCCHIO_THROW(
          hasCollisionModel(), std::logic_error, "No collision model in the visualizer.");
        return *m_collisionModel;
      }

      bool hasCollisionModel() const
      {
        return m_collisionModel != nullptr;
      }

    protected:
      std::reference_wrapper<Model const> m_model;
      GeometryModel const * m_visualModel;
      GeometryModel const * m_collisionModel;

      /// @brief This method is called at the beginning of display().
      virtual void displayPrecall()
      {
      }
      virtual void displayImpl() = 0;

    public:
      Data data;
      GeometryData visualData;
      GeometryData collisionData;
    };
  } // namespace visualizers
} // namespace pinocchio

namespace pinviz = ::pinocchio::visualizers; // NOLINT

#endif // ifndef __pinocchio_extra_base_visualizer_hxx__
