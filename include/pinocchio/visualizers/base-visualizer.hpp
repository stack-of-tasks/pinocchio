//
// Copyright (c) 2024-2025 INRIA
//

#ifndef __pinocchio_extra_base_visualizer_hpp__
#define __pinocchio_extra_base_visualizer_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/visualizers/config.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include <boost/optional.hpp>
#include <utility>

namespace pinocchio
{
  namespace visualizers
  {
    typedef PINOCCHIO_SCALAR_TYPE_DEFAULT Scalar;
    PINOCCHIO_COMMON_TYPEDEF(PINOCCHIO_SCALAR_TYPE_DEFAULT, PINOCCHIO_OPTIONS_DEFAULT)

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
    class PINOCCHIO_VISUALIZERS_DLLAPI BaseVisualizer
    {
    public:
      typedef SE3::Matrix4 Matrix4;

      /// @brief Class constructor for borrowing external data.
      ///
      /// @param model Reference to Pinocchio Model class.
      /// @param visual_model Reference to visual GeometryModel.
      /// @param collision_model Pointer to collision model. Pass nullptr if no collision model is
      /// to be provided.
      /// @param data Reference to Data object. The visualizer will store an internal reference as a
      /// pointer.
      /// @param visual_data Reference to visual (VISUAL) GeometryData object. The visualizer will
      /// store an internal reference as a pointer.
      /// @param collision_data Pointer to collision (COLLISION) GeometryData object. The visualizer
      /// will store this pointer internally as a reference.
      /// @remark This constructor throws if a \p collision_model is provided with no \p
      /// collision_data. If the reverse happens, then the collision GeometryData is simply ignored.
      BaseVisualizer(
        const Model & model,
        const GeometryModel & visual_model,
        const GeometryModel * collision_model,
        Data & data,
        GeometryData & visual_data,
        GeometryData * collision_data);

      /// @brief Class constructor which will create internally-managed data objects.
      ///
      /// @param model Reference to Pinocchio Model class.
      /// @param visual_model Reference to visual GeometryModel.
      /// @param collision_model Pointer to collision model. Pass nullptr if no collision model is
      /// to be provided.
      BaseVisualizer(
        const Model & model,
        const GeometryModel & visual_model,
        const GeometryModel * collision_model = nullptr);

      virtual ~BaseVisualizer();

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

      /// @copybrief display()
      template<typename D>
      void display(const Eigen::MatrixBase<D> & q)
      {
        boost::optional<ConstVectorRef> q_(q);
        display(q_);
      }

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

      /// @brief Whether the internal data pointers are borrowed (external), or owned.
      bool hasExternalData() const
      {
        return !m_ownedData;
      }

      Data & data()
      {
        return *m_data;
      }
      const Data & data() const
      {
        return *m_data;
      }

      GeometryData & visualData()
      {
        return *m_visualData;
      }
      const GeometryData & visualData() const
      {
        return *m_visualData;
      }

      GeometryData & collisionData()
      {
        PINOCCHIO_THROW(
          hasCollisionModel(), std::logic_error, "No collision model in the visualizer.");
        return *m_collisionData;
      }

      const GeometryData & collisionData() const
      {
        PINOCCHIO_THROW(
          hasCollisionModel(), std::logic_error, "No collision model in the visualizer.");
        return *m_collisionData;
      }

    protected:
      std::reference_wrapper<Model const> m_model;
      GeometryModel const * m_visualModel;
      GeometryModel const * m_collisionModel;

      Data * m_data;
      GeometryData * m_visualData;
      GeometryData * m_collisionData;
      bool m_ownedData;

      /// @brief This method is called at the beginning of display().
      virtual void displayPrecall()
      {
      }
      virtual void displayImpl() = 0;

      void destroyData();
    };
  } // namespace visualizers
} // namespace pinocchio

#endif // ifndef __pinocchio_extra_base_visualizer_hxx__
