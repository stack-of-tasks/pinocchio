//
// Copyright (c) 2015-2023 CNRS INRIA
//

#ifndef __pinocchio_parsers_urdf_types_hpp__
#define __pinocchio_parsers_urdf_types_hpp__

#include <urdf_model/model.h>

#include <string>
#include <exception>
#ifdef PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
#include <memory>
#define PINOCCHIO_URDF_SHARED_PTR(type) std::shared_ptr<type>
#define PINOCCHIO_URDF_WEAK_PTR(type) std::weak_ptr<type>
#else
#include <boost/shared_ptr.hpp>
#define PINOCCHIO_URDF_SHARED_PTR(type) boost::shared_ptr<type>
#define PINOCCHIO_URDF_WEAK_PTR(type) boost::weak_ptr<type>
#endif

#ifndef PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR

#define PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Class) \
typedef PINOCCHIO_URDF_SHARED_PTR(Class) Class##SharedPtr; \
typedef PINOCCHIO_URDF_SHARED_PTR(const Class) Class##ConstSharedPtr; \
typedef PINOCCHIO_URDF_WEAK_PTR(Class) Class##WeakPtr

namespace urdf
{
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Box);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Collision);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Cylinder);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Geometry);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Inertial);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Joint);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Link);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Material);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Mesh);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(ModelInterface);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Sphere);
  PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER(Visual);
  
  template<class T, class U>
  PINOCCHIO_URDF_SHARED_PTR(T) const_pointer_cast(PINOCCHIO_URDF_SHARED_PTR(U) const & r)
  {
#ifdef PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
    return std::const_pointer_cast<T>(r);
#else
    return boost::const_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  PINOCCHIO_URDF_SHARED_PTR(T) dynamic_pointer_cast(PINOCCHIO_URDF_SHARED_PTR(U) const & r)
  {
#ifdef PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
    return std::dynamic_pointer_cast<T>(r);
#else
    return boost::dynamic_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  PINOCCHIO_URDF_SHARED_PTR(T) static_pointer_cast(PINOCCHIO_URDF_SHARED_PTR(U) const & r)
  {
#ifdef PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
    return std::static_pointer_cast<T>(r);
#else
    return boost::static_pointer_cast<T>(r);
#endif
  }
}

#undef PINOCCHIO_URDF_TYPEDEF_CLASS_POINTER

#else // PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR

#include <urdf_world/types.h>

#endif // PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR

#endif // ifndef __pinocchio_parsers_urdf_types_hpp__
