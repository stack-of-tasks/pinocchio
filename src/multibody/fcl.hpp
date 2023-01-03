//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_multibody_fcl_hpp__
#define __pinocchio_multibody_fcl_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL

  #if(WIN32)
    // It appears that std::snprintf is missing for Windows.
    #if !(( defined(_MSC_VER) && _MSC_VER < 1900 ) || ( defined(__MINGW32__) && !defined(__MINGW64_VERSION_MAJOR) ))
      #include <cstdio>
      #include <stdarg.h>
      namespace std
      {
        inline int _snprintf(char* buffer, std::size_t buf_size, const char* format, ...)
        {
          int res;
          
          va_list args;
          va_start(args, format);
          res = vsnprintf(buffer, buf_size, format, args);
          va_end(args);
          
          return res;
        }
      }
    #endif
  #endif
  
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

#ifdef PINOCCHIO_WITH_HPP_FCL
#if HPP_FCL_VERSION_AT_LEAST(2,0,0)
#define PINOCCHIO_HPPFCL_USE_STD_SHARED_PTR
#endif
#endif

#ifdef PINOCCHIO_HPPFCL_USE_STD_SHARED_PTR
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#endif

#include <boost/foreach.hpp>

namespace pinocchio
{
#ifdef PINOCCHIO_HPPFCL_USE_STD_SHARED_PTR
  using std::shared_ptr;
  using std::make_shared;
#else
  using boost::shared_ptr;
  using boost::make_shared;
#endif
  struct CollisionPair
  : public std::pair<GeomIndex, GeomIndex>
  {

    typedef std::pair<GeomIndex, GeomIndex> Base;
   
    /// \brief Empty constructor
    CollisionPair();
    
    ///
    /// \brief Default constructor of a collision pair from two collision object indexes.
    /// \remarks The two indexes must be different, otherwise the constructor throws.
    ///
    /// \param[in] co1 Index of the first collision object.
    /// \param[in] co2 Index of the second collision object.
    ///
    CollisionPair(const GeomIndex co1, const GeomIndex co2);
    bool                  operator == (const CollisionPair& rhs) const;
    bool                  operator != (const CollisionPair& rhs) const;
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
  
  typedef shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
  
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
  
  /// \brief If true, no collision or distance check will be done between the Geometry and any other geometry
  bool disableCollision;

PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
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
                 const Eigen::Vector4d & meshColor = Eigen::Vector4d(0,0,0,1),
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
  , disableCollision(false)
  {}
PINOCCHIO_COMPILER_DIAGNOSTIC_POP

PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
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
                 const Eigen::Vector4d & meshColor = Eigen::Vector4d::Ones(),
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
  , disableCollision(false)
  {}
PINOCCHIO_COMPILER_DIAGNOSTIC_POP

PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  GeometryObject(const GeometryObject & other)
  : fcl(geometry)
  {
    *this = other;
    
  }
PINOCCHIO_COMPILER_DIAGNOSTIC_POP

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
    disableCollision   = other.disableCollision;
    return *this;
  }

  friend std::ostream & operator<< (std::ostream & os, const GeometryObject & geomObject);
};

#ifdef PINOCCHIO_WITH_HPP_FCL

  struct ComputeCollision
  : ::hpp::fcl::ComputeCollision
  {
    typedef ::hpp::fcl::ComputeCollision Base;
    typedef shared_ptr<const fcl::CollisionGeometry> ConstCollisionGeometryPtr;
    
    ComputeCollision(const GeometryObject & o1, const GeometryObject & o2)
    : Base(o1.geometry.get(),o2.geometry.get())
    , o1(o1.geometry)
    , o2(o2.geometry)
    {}
    
    virtual ~ComputeCollision() {};
    
  protected:
    ConstCollisionGeometryPtr o1;
    ConstCollisionGeometryPtr o2;
    
    virtual std::size_t run(const fcl::Transform3f& tf1, const fcl::Transform3f& tf2,
                            const fcl::CollisionRequest& request, fcl::CollisionResult& result) const
    {
      typedef ::hpp::fcl::CollisionGeometry const * Pointer;
      const_cast<Pointer&>(Base::o1) = o1.get();
      const_cast<Pointer&>(Base::o2) = o2.get();
      return Base::run(tf1, tf2, request, result);
    }
  };

  struct ComputeDistance
  : ::hpp::fcl::ComputeDistance
  {
    typedef ::hpp::fcl::ComputeDistance Base;
    typedef shared_ptr<fcl::CollisionGeometry> ConstCollisionGeometryPtr;
    
    ComputeDistance(const GeometryObject & o1, const GeometryObject & o2)
    : Base(o1.geometry.get(),o2.geometry.get())
    , o1(o1.geometry)
    , o2(o2.geometry)
    {}
    
    virtual ~ComputeDistance() {};
    
  protected:
    ConstCollisionGeometryPtr o1;
    ConstCollisionGeometryPtr o2;
    
    virtual hpp::fcl::FCL_REAL run(const fcl::Transform3f& tf1, const fcl::Transform3f& tf2,
                                   const fcl::DistanceRequest& request, fcl::DistanceResult& result) const
    {
      typedef ::hpp::fcl::CollisionGeometry const * Pointer;
      const_cast<Pointer&>(Base::o1) = o1.get();
      const_cast<Pointer&>(Base::o2) = o2.get();
      return Base::run(tf1, tf2, request, result);
    }
  };
  
#endif

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/fcl.hxx"

#endif // ifndef __pinocchio_multibody_fcl_hpp__
