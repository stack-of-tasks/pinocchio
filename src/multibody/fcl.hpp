//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_fcl_hpp__
#define __pinocchio_fcl_hpp__

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
  
  /// \brief Name of the geometry object
  std::string name;

  /// \brief Index of the parent frame
  ///
  /// Parent frame may be unset (set to -1) as it is mostly used as a documentation of the tree, or in third-party libraries.
  /// The URDF parser of Pinocchio is setting it to the proper value according to the urdf link-joint tree.
  /// In particular, anchor joints of URDF would cause parent frame to be different to joint frame.
  FrameIndex parentFrame;

  /// \brief Index of the parent joint
  JointIndex parentJoint;

  /// \brief The actual cloud of points representing the collision mesh of the object after scaling.
  boost::shared_ptr<fcl::CollisionGeometry> fcl;

  /// \brief Position of geometry object in parent joint frame
  SE3 placement;

  /// \brief Absolute path to the mesh file
  std::string meshPath;

  /// \brief Scaling vector applied to the fcl object.
  Eigen::Vector3d meshScale;

  /// \brief Decide whether to override the Material.
  bool overrideMaterial;

  /// \brief RGBA color value of the mesh.
  Eigen::Vector4d meshColor;

  /// \brief Absolute path to the mesh texture file.
  std::string meshTexturePath;

  GeometryObject(const std::string & name, const FrameIndex parentF,
                 const JointIndex parentJ,
                 const boost::shared_ptr<fcl::CollisionGeometry> & collision,
                 const SE3 & placement, const std::string & meshPath = "",
                 const Eigen::Vector3d & meshScale = Eigen::Vector3d::Ones(),
                 const bool overrideMaterial = false,
                 const Eigen::Vector4d & meshColor = Eigen::Vector4d::Zero(),
                 const std::string & meshTexturePath = "")
                : name(name)
                , parentFrame(parentF)
                , parentJoint(parentJ)
                , fcl(collision)
                , placement(placement)
                , meshPath(meshPath)
                , meshScale(meshScale)
                , overrideMaterial(overrideMaterial)
                , meshColor(meshColor)
                , meshTexturePath(meshTexturePath)
  {}

  GeometryObject & operator=(const GeometryObject & other)
  {
    name                = other.name;
    parentFrame         = other.parentFrame;
    parentJoint         = other.parentJoint;
    fcl                 = other.fcl;
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


#endif // ifndef __pinocchio_fcl_hpp__
