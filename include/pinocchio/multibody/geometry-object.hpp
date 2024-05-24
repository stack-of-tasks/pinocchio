//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_multibody_geometry_object_hpp__
#define __pinocchio_multibody_geometry_object_hpp__

#include "pinocchio/utils/shared-ptr.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model-item.hpp"
#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/serialization/serializable.hpp"

/// Be carefull to include this header after fwd.hpp.
/// fwd.hpp contains some define to change the boost::variant max size.
/// If we don't include it before, default size is choosed that can
/// make all the build fail.
#include <boost/variant.hpp>

namespace pinocchio
{

  enum GeometryType
  {
    VISUAL,
    COLLISION
  };

  /// No material associated to a geometry.
  struct GeometryNoMaterial
  {
    bool operator==(const GeometryNoMaterial &) const
    {
      return true;
    }
  };

  /// Mesh material based on the Phong lighting model.
  /// Diffuse color is stored in \p GeometryObject::meshColor.
  struct GeometryPhongMaterial
  {
    GeometryPhongMaterial() = default;
    GeometryPhongMaterial(
      const Eigen::Vector4d & meshEmissionColor,
      const Eigen::Vector4d & meshSpecularColor,
      double meshShininess)
    : meshEmissionColor(meshEmissionColor)
    , meshSpecularColor(meshSpecularColor)
    , meshShininess(meshShininess)
    {
    }

    bool operator==(const GeometryPhongMaterial & other) const
    {
      return meshEmissionColor == other.meshEmissionColor
             && meshSpecularColor == other.meshSpecularColor
             && meshShininess == other.meshShininess;
    }

    /// \brief RGBA emission (ambient) color value of the GeometryObject::geometry object.
    Eigen::Vector4d meshEmissionColor{Eigen::Vector4d(0., 0., 0., 1.)};

    /// \brief RGBA specular color value of the GeometryObject::geometry object.
    Eigen::Vector4d meshSpecularColor{Eigen::Vector4d(0., 0., 0., 1.)};

    /// \brief Shininess associated to the specular lighting model.
    ///
    /// This value must normalized between 0 and 1.
    double meshShininess{0.};
  };

  typedef boost::variant<GeometryNoMaterial, GeometryPhongMaterial> GeometryMaterial;

  struct GeometryObject; // fwd

  template<>
  struct traits<GeometryObject>
  {
    typedef context::Scalar Scalar;
    enum
    {
      Options = context::Options
    };
  };

  struct GeometryObject
  : public ModelItem<GeometryObject>
  , serialization::Serializable<GeometryObject>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ModelItem<GeometryObject> Base;
    typedef typename traits<GeometryObject>::Scalar Scalar;
    enum
    {
      Options = traits<GeometryObject>::Options
    };

    typedef SE3Tpl<Scalar, Options> SE3;

    typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;

    using Base::name;
    using Base::parentFrame;
    using Base::parentJoint;
    using Base::placement;

    /// \brief The FCL CollisionGeometry (might be a Mesh, a Geometry Primitive, etc.)
    CollisionGeometryPtr geometry;

    /// \brief Absolute path to the mesh file (if the geometry pointee is also a Mesh)
    std::string meshPath;

    /// \brief Scaling vector applied to the GeometryObject::geometry object.
    Eigen::Vector3d meshScale;

    /// \brief Decide whether to override the Material.
    bool overrideMaterial;

    /// \brief RGBA color value of the GeometryObject::geometry object.
    Eigen::Vector4d meshColor;

    /// \brief Material associated to the mesh.
    /// This material should be used only if overrideMaterial is set to true.
    /// In other case, the mesh default material must be used.
    GeometryMaterial meshMaterial;

    /// \brief Absolute path to the mesh texture file.
    std::string meshTexturePath;

    /// \brief If true, no collision or distance check will be done between the Geometry and any
    /// other geometry
    bool disableCollision;

    ///
    /// \brief Full constructor.
    ///
    /// \param[in] name  Name of the geometry object.
    /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
    /// \param[in] parent_frame  Index of the parent frame.
    /// \param[in] placement Placement of the geometry with respect to the joint frame.
    /// \param[in] collision_geometry The FCL collision geometry object.
    /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a
    /// viewer for instance) [if applicable]. \param[in] meshScale Scale of the mesh [if
    /// applicable]. \param[in] overrideMaterial If true, this option allows to overrite the
    /// material [if applicable]. \param[in] meshColor Color of the mesh [if applicable]. \param[in]
    /// meshTexturePath Path to the file containing the texture information [if applicable].
    /// \param[in] meshMaterial Material of the mesh [if applicable].
    ///
    GeometryObject(
      const std::string & name,
      const JointIndex parent_joint,
      const FrameIndex parent_frame,
      const SE3 & placement,
      const CollisionGeometryPtr & collision_geometry,
      const std::string & meshPath = "",
      const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
      const bool overrideMaterial = false,
      const Eigen::Vector4d & meshColor = Eigen::Vector4d(0, 0, 0, 1),
      const std::string & meshTexturePath = "",
      const GeometryMaterial & meshMaterial = GeometryNoMaterial())
    : Base(name, parent_joint, parent_frame, placement)
    , geometry(collision_geometry)
    , meshPath(meshPath)
    , meshScale(meshScale)
    , overrideMaterial(overrideMaterial)
    , meshColor(meshColor)
    , meshMaterial(meshMaterial)
    , meshTexturePath(meshTexturePath)
    , disableCollision(false)
    {
    }

    ///
    /// \brief Reduced constructor.
    /// \remarks Compared to the other constructor, this one assumes that there is no parentFrame
    /// associated to the geometry.
    ///
    /// \param[in] name  Name of the geometry object.
    /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
    /// \param[in] placement Placement of the geometry with respect to the joint frame.
    /// \param[in] collision_geometry The FCL collision geometry object.
    /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a
    /// viewer for instance) [if applicable]. \param[in] meshScale Scale of the mesh [if
    /// applicable]. \param[in] overrideMaterial If true, this option allows to overrite the
    /// material [if applicable]. \param[in] meshColor Color of the mesh [if applicable]. \param[in]
    /// meshTexturePath Path to the file containing the texture information [if applicable].
    /// \param[in] meshMaterial Material of the mesh [if applicable].
    ///
    GeometryObject(
      const std::string & name,
      const JointIndex parent_joint,
      const SE3 & placement,
      const CollisionGeometryPtr & collision_geometry,
      const std::string & meshPath = "",
      const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
      const bool overrideMaterial = false,
      const Eigen::Vector4d & meshColor = Eigen::Vector4d(0, 0, 0, 1),
      const std::string & meshTexturePath = "",
      const GeometryMaterial & meshMaterial = GeometryNoMaterial())
    : Base(name, parent_joint, std::numeric_limits<FrameIndex>::max(), placement)
    , geometry(collision_geometry)
    , meshPath(meshPath)
    , meshScale(meshScale)
    , overrideMaterial(overrideMaterial)
    , meshColor(meshColor)
    , meshMaterial(meshMaterial)
    , meshTexturePath(meshTexturePath)
    , disableCollision(false)
    {
    }

    ///
    /// \brief Full constructor.
    ///
    /// \param[in] name  Name of the geometry object.
    /// \param[in] parent_frame  Index of the parent frame.
    /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
    /// \param[in] collision_geometry The FCL collision geometry object.
    /// \param[in] placement Placement of the geometry with respect to the joint frame.
    /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a
    /// viewer for instance) [if applicable]. \param[in] meshScale Scale of the mesh [if
    /// applicable]. \param[in] overrideMaterial If true, this option allows to overrite the
    /// material [if applicable]. \param[in] meshColor Color of the mesh [if applicable]. \param[in]
    /// meshTexturePath Path to the file containing the texture information [if applicable].
    /// \param[in] meshMaterial Material of the mesh [if applicable].
    ///
    /// \deprecated This constructor is now deprecated, and its argument order has been changed.
    ///
    PINOCCHIO_DEPRECATED GeometryObject(
      const std::string & name,
      const FrameIndex parent_frame,
      const JointIndex parent_joint,
      const CollisionGeometryPtr & collision_geometry,
      const SE3 & placement,
      const std::string & meshPath = "",
      const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
      const bool overrideMaterial = false,
      const Eigen::Vector4d & meshColor = Eigen::Vector4d(0, 0, 0, 1),
      const std::string & meshTexturePath = "",
      const GeometryMaterial & meshMaterial = GeometryNoMaterial())
    : Base(name, parent_joint, parent_frame, placement)
    , geometry(collision_geometry)
    , meshPath(meshPath)
    , meshScale(meshScale)
    , overrideMaterial(overrideMaterial)
    , meshColor(meshColor)
    , meshMaterial(meshMaterial)
    , meshTexturePath(meshTexturePath)
    , disableCollision(false)
    {
    }

    ///
    /// \brief Reduced constructor.
    /// \remarks Compared to the other constructor, this one assumes that there is no parentFrame
    /// associated to the geometry.
    ///
    /// \param[in] name  Name of the geometry object.
    /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
    /// \param[in] collision_geometry The FCL collision geometry object.
    /// \param[in] placement Placement of the geometry with respect to the joint frame.
    /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a
    /// viewer for instance) [if applicable]. \param[in] meshScale Scale of the mesh [if
    /// applicable]. \param[in] overrideMaterial If true, this option allows to overrite the
    /// material [if applicable]. \param[in] meshColor Color of the mesh [if applicable]. \param[in]
    /// meshTexturePath Path to the file containing the texture information [if applicable].
    /// \param[in] meshMaterial Material of the mesh [if applicable].
    ///
    /// \deprecated This constructor is now deprecated, and its argument order has been changed.
    ///
    PINOCCHIO_DEPRECATED GeometryObject(
      const std::string & name,
      const JointIndex parent_joint,
      const CollisionGeometryPtr & collision_geometry,
      const SE3 & placement,
      const std::string & meshPath = "",
      const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
      const bool overrideMaterial = false,
      const Eigen::Vector4d & meshColor = Eigen::Vector4d(0, 0, 0, 1),
      const std::string & meshTexturePath = "",
      const GeometryMaterial & meshMaterial = GeometryNoMaterial())
    : Base(name, parent_joint, std::numeric_limits<FrameIndex>::max(), placement)
    , geometry(collision_geometry)
    , meshPath(meshPath)
    , meshScale(meshScale)
    , overrideMaterial(overrideMaterial)
    , meshColor(meshColor)
    , meshMaterial(meshMaterial)
    , meshTexturePath(meshTexturePath)
    , disableCollision(false)
    {
    }

    GeometryObject(const GeometryObject & other) = default;
    GeometryObject & operator=(const GeometryObject & other) = default;

    ///
    /// \brief Perform a deep copy of this. It will create a copy of the underlying FCL geometry.
    ///
    GeometryObject clone() const
    {
      GeometryObject res(*this);

#ifdef PINOCCHIO_WITH_HPP_FCL
      if (geometry)
        res.geometry = CollisionGeometryPtr(geometry->clone());
#endif

      return res;
    }

    bool operator==(const GeometryObject & other) const
    {
      if (this == &other)
        return true;
      return name == other.name && parentFrame == other.parentFrame
             && parentJoint == other.parentJoint && placement == other.placement
             && meshPath == other.meshPath && meshScale == other.meshScale
             && overrideMaterial == other.overrideMaterial && meshColor == other.meshColor
             && meshMaterial == other.meshMaterial && meshTexturePath == other.meshTexturePath
             && disableCollision == other.disableCollision
             && compare_shared_ptr(geometry, other.geometry);
    }

    bool operator!=(const GeometryObject & other) const
    {
      return !(*this == other);
    }

    friend std::ostream & operator<<(std::ostream & os, const GeometryObject & geomObject);
  };

#ifdef PINOCCHIO_WITH_HPP_FCL

  struct CollisionObject : ::hpp::fcl::CollisionObject
  {
    typedef ::hpp::fcl::CollisionObject Base;
    typedef SE3Tpl<double> SE3;

    CollisionObject()
    : Base(nullptr, false)
    , geometryObjectIndex((std::numeric_limits<size_t>::max)())
    {
    }

    explicit CollisionObject(
      const std::shared_ptr<::hpp::fcl::CollisionGeometry> & collision_geometry,
      const size_t geometryObjectIndex = (std::numeric_limits<size_t>::max)(),
      bool compute_local_aabb = true)
    : Base(collision_geometry, compute_local_aabb)
    , geometryObjectIndex(geometryObjectIndex)
    {
    }

    CollisionObject(
      const std::shared_ptr<::hpp::fcl::CollisionGeometry> & collision_geometry,
      const SE3 & transform,
      const size_t geometryObjectIndex = (std::numeric_limits<size_t>::max)(),
      bool compute_local_aabb = true)
    : Base(collision_geometry, toFclTransform3f(transform), compute_local_aabb)
    , geometryObjectIndex(geometryObjectIndex)
    {
    }

    bool operator==(const CollisionObject & other) const
    {
      return Base::operator==(other) && geometryObjectIndex == other.geometryObjectIndex;
    }

    bool operator!=(const CollisionObject & other) const
    {
      return !(*this == other);
    }

    /// @brief Geometry object index related to the current collision object.
    size_t geometryObjectIndex;
  };

  struct ComputeCollision : ::hpp::fcl::ComputeCollision
  {
    typedef ::hpp::fcl::ComputeCollision Base;

    ComputeCollision(const GeometryObject & go1, const GeometryObject & go2)
    : Base(go1.geometry.get(), go2.geometry.get())
    , go1_ptr(&go1)
    , go2_ptr(&go2)
    {
    }

    virtual ~ComputeCollision() {};

    virtual std::size_t run(
      const fcl::Transform3f & tf1,
      const fcl::Transform3f & tf2,
      const fcl::CollisionRequest & request,
      fcl::CollisionResult & result) const
    {
      typedef ::hpp::fcl::CollisionGeometry const * Pointer;
      const_cast<Pointer &>(Base::o1) = go1_ptr->geometry.get();
      const_cast<Pointer &>(Base::o2) = go2_ptr->geometry.get();
      return Base::run(tf1, tf2, request, result);
    }

    bool operator==(const ComputeCollision & other) const
    {
      return Base::operator==(other) && go1_ptr == other.go1_ptr
             && go2_ptr == other.go2_ptr; // Maybe, it would be better to just check *go2_ptr ==
                                          // *other.go2_ptr
    }

    bool operator!=(const ComputeCollision & other) const
    {
      return !(*this == other);
    }

    const GeometryObject & getGeometryObject1() const
    {
      return *go1_ptr;
    }
    const GeometryObject & getGeometryObject2() const
    {
      return *go2_ptr;
    }

  protected:
    const GeometryObject * go1_ptr;
    const GeometryObject * go2_ptr;
  };

  struct ComputeDistance : ::hpp::fcl::ComputeDistance
  {
    typedef ::hpp::fcl::ComputeDistance Base;

    ComputeDistance(const GeometryObject & go1, const GeometryObject & go2)
    : Base(go1.geometry.get(), go2.geometry.get())
    , go1_ptr(&go1)
    , go2_ptr(&go2)
    {
    }

    virtual ~ComputeDistance() {};

    virtual hpp::fcl::FCL_REAL run(
      const fcl::Transform3f & tf1,
      const fcl::Transform3f & tf2,
      const fcl::DistanceRequest & request,
      fcl::DistanceResult & result) const
    {
      typedef ::hpp::fcl::CollisionGeometry const * Pointer;
      const_cast<Pointer &>(Base::o1) = go1_ptr->geometry.get();
      const_cast<Pointer &>(Base::o2) = go2_ptr->geometry.get();
      return Base::run(tf1, tf2, request, result);
    }

    bool operator==(const ComputeDistance & other) const
    {
      return Base::operator==(other) && go1_ptr == other.go1_ptr && go2_ptr == other.go2_ptr;
    }

    bool operator!=(const ComputeDistance & other) const
    {
      return !(*this == other);
    }

    const GeometryObject & getGeometryObject1() const
    {
      return *go1_ptr;
    }
    const GeometryObject & getGeometryObject2() const
    {
      return *go2_ptr;
    }

  protected:
    const GeometryObject * go1_ptr;
    const GeometryObject * go2_ptr;
  };

#endif

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry-object.hxx"

#endif // ifndef __pinocchio_multibody_geometry_object_hpp__
