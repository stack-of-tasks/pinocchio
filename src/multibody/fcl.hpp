//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_multibody_fcl_hpp__
#define __pinocchio_multibody_fcl_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include <hpp/fcl/collision_object.h>
  #include <hpp/fcl/collision.h>
  #include <hpp/fcl/distance.h>
  #include <hpp/fcl/shape/geometric_shapes.h>
  #include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#endif

#include <iostream>
#include <map>
#include <vector>
#include <utility>
#include <limits>
#include <assert.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace pinocchio
{
  struct CollisionPair: public std::pair<GeomIndex, GeomIndex>
  {

    typedef std::pair<GeomIndex, GeomIndex> Base;
   
    CollisionPair(const GeomIndex co1, const GeomIndex co2);
    bool                  operator == (const CollisionPair& rhs) const;
    void                  disp        (std::ostream & os)        const;
    friend std::ostream & operator << (std::ostream & os,const CollisionPair & X);

  }; // struct CollisionPair

#ifndef PINOCCHIO_WITH_HPP_FCL  

  namespace fcl
  {
 
    struct FakeCollisionGeometry
    {
      FakeCollisionGeometry(){};
      
      bool operator==(const FakeCollisionGeometry &) const
      {
        return true;
      }
    };

    struct AABB
    {
      AABB(): min_(0), max_(1){};

      int min_;
      int max_;
    };
  
    typedef FakeCollisionGeometry CollisionGeometry;

  }

#else

  namespace fcl = hpp::fcl;

#endif // PINOCCHIO_WITH_HPP_FCL

enum GeometryType
{
  VISUAL,
  COLLISION
};

struct GeometryObject
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef boost::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
  
  /// \brief Name of the geometry object
  std::string name;

  /// \brief Index of the parent frame
  ///
  /// Parent frame may be unset (to the std::numeric_limits<FrameIndex>::max() value) as it is mostly used as a documentation of the tree, or in third-party libraries.
  /// The URDF parser of Pinocchio is setting it to the proper value according to the urdf link-joint tree.
  /// In particular, anchor joints of URDF would cause parent frame to be different to joint frame.
  FrameIndex parentFrame;

  /// \brief Index of the parent joint
  JointIndex parentJoint;

  /// \brief The FCL CollisionGeometry (might be a Mesh, a Geometry Primitive, etc.)
  CollisionGeometryPtr geometry;

  /// \brief The former pointer to the FCL CollisionGeometry.
  /// \deprecated It is now deprecated and has been renamed GeometryObject::geometry
  PINOCCHIO_DEPRECATED CollisionGeometryPtr & fcl;

  /// \brief Position of geometry object in parent joint frame
  SE3 placement;

  /// \brief Absolute path to the mesh file (if the fcl pointee is also a Mesh)
  std::string meshPath;

  /// \brief Scaling vector applied to the GeometryObject::fcl object.
  Eigen::Vector3d meshScale;

  /// \brief Decide whether to override the Material.
  bool overrideMaterial;

  /// \brief RGBA color value of the GeometryObject::fcl object.
  Eigen::Vector4d meshColor;

  /// \brief Absolute path to the mesh texture file.
  std::string meshTexturePath;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ///
  /// \brief Full constructor.
  ///
  /// \param[in] name  Name of the geometry object.
  /// \param[in] parent_frame  Index of the parent frame.
  /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
  /// \param[in] collision_geometry The FCL collision geometry object.
  /// \param[in] placement Placement of the geometry with respect to the joint frame.
  /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a viewer for instance) [if applicable].
  /// \param[in] meshScale Scale of the mesh [if applicable].
  /// \param[in] overrideMaterial If true, this option allows to overrite the material [if applicable].
  /// \param[in] meshColor Color of the mesh [if applicable].
  /// \param[in] meshTexturePath Path to the file containing the texture information [if applicable].
  ///
  GeometryObject(const std::string & name,
                 const FrameIndex parent_frame,
                 const JointIndex parent_joint,
                 const CollisionGeometryPtr & collision_geometry,
                 const SE3 & placement,
                 const std::string & meshPath = "",
                 const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
                 const bool overrideMaterial = false,
                 const Eigen::Vector4d & meshColor = Eigen::Vector4d::Zero(),
                 const std::string & meshTexturePath = "")
  : name(name)
  , parentFrame(parent_frame)
  , parentJoint(parent_joint)
  , geometry(collision_geometry)
  , fcl(geometry)
  , placement(placement)
  , meshPath(meshPath)
  , meshScale(meshScale)
  , overrideMaterial(overrideMaterial)
  , meshColor(meshColor)
  , meshTexturePath(meshTexturePath)
  {}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ///
  /// \brief Reduced constructor.
  /// \remarks Compared to the other constructor, this one assumes that there is no parentFrame associated to the geometry.
  ///
  /// \param[in] name  Name of the geometry object.
  /// \param[in] parent_joint  Index of the parent joint (that supports the geometry).
  /// \param[in] collision_geometry The FCL collision geometry object.
  /// \param[in] placement Placement of the geometry with respect to the joint frame.
  /// \param[in] meshPath Path of the mesh (may be needed extarnally to load the mesh inside a viewer for instance) [if applicable].
  /// \param[in] meshScale Scale of the mesh [if applicable].
  /// \param[in] overrideMaterial If true, this option allows to overrite the material [if applicable].
  /// \param[in] meshColor Color of the mesh [if applicable].
  /// \param[in] meshTexturePath Path to the file containing the texture information [if applicable].
  ///
  GeometryObject(const std::string & name,
                 const JointIndex parent_joint,
                 const CollisionGeometryPtr & collision_geometry,
                 const SE3 & placement,
                 const std::string & meshPath = "",
                 const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
                 const bool overrideMaterial = false,
                 const Eigen::Vector4d & meshColor = Eigen::Vector4d::Zero(),
                 const std::string & meshTexturePath = "")
  : name(name)
  , parentFrame(std::numeric_limits<FrameIndex>::max())
  , parentJoint(parent_joint)
  , geometry(collision_geometry)
  , fcl(geometry)
  , placement(placement)
  , meshPath(meshPath)
  , meshScale(meshScale)
  , overrideMaterial(overrideMaterial)
  , meshColor(meshColor)
  , meshTexturePath(meshTexturePath)
  {}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  GeometryObject(const GeometryObject & other)
  : fcl(geometry)
  {
    *this = other;
  }
#pragma GCC diagnostic pop

  GeometryObject & operator=(const GeometryObject & other)
  {
    name                = other.name;
    parentFrame         = other.parentFrame;
    parentJoint         = other.parentJoint;
    geometry            = other.geometry;
    placement           = other.placement;
    meshPath            = other.meshPath;
    meshScale           = other.meshScale;
    overrideMaterial    = other.overrideMaterial;
    meshColor           = other.meshColor;
    meshTexturePath     = other.meshTexturePath;
    return *this;
  }

  friend std::ostream & operator<< (std::ostream & os, const GeometryObject & geomObject);
};
  

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/fcl.hxx"

#endif // ifndef __pinocchio_multibody_fcl_hpp__
