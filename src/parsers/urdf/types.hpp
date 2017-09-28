//
// Copyright (c) 2015-2017 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_parsers_urdf_types_hpp__
#define __se3_parsers_urdf_types_hpp__

#include <urdf_model/model.h>

#include <string>
#include <exception>
#ifdef URDFDOM_USE_STD_SHARED_PTR
#include <memory>
#define URDF_SHARED_PTR(type) std::shared_ptr<type>
#define URDF_WEAK_PTR(type) std::weak_ptr<type>
#else
#include <boost/shared_ptr.hpp>
#define URDF_SHARED_PTR(type) boost::shared_ptr<type>
#define URDF_WEAK_PTR(type) boost::weak_ptr<type>
#endif

#ifndef URDFDOM_TYPEDEF_SHARED_PTR

#define URDF_TYPEDEF_CLASS_POINTER(Class) \
typedef URDF_SHARED_PTR(Class) Class##SharedPtr; \
typedef URDF_SHARED_PTR(const Class) Class##ConstSharedPtr; \
typedef URDF_WEAK_PTR(Class) Class##WeakPtr

namespace urdf
{
  URDF_TYPEDEF_CLASS_POINTER(Box);
  URDF_TYPEDEF_CLASS_POINTER(Collision);
  URDF_TYPEDEF_CLASS_POINTER(Cylinder);
  URDF_TYPEDEF_CLASS_POINTER(Geometry);
  URDF_TYPEDEF_CLASS_POINTER(Inertial);
  URDF_TYPEDEF_CLASS_POINTER(Joint);
  URDF_TYPEDEF_CLASS_POINTER(Link);
  URDF_TYPEDEF_CLASS_POINTER(Material);
  URDF_TYPEDEF_CLASS_POINTER(Mesh);
  URDF_TYPEDEF_CLASS_POINTER(ModelInterface);
  URDF_TYPEDEF_CLASS_POINTER(Sphere);
  URDF_TYPEDEF_CLASS_POINTER(Visual);
  
  template<class T, class U>
  URDF_SHARED_PTR(T) const_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std::const_pointer_cast<T>(r);
#else
    return boost::const_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  URDF_SHARED_PTR(T) dynamic_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std::dynamic_pointer_cast<T>(r);
#else
    return boost::dynamic_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  URDF_SHARED_PTR(T) static_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std:static_pointer_cast<T>(r);
#else
    return boost::static_pointer_cast<T>(r);
#endif
  }
}

#undef URDF_TYPEDEF_CLASS_POINTER

#else // URDFDOM_TYPEDEF_SHARED_PTR

#include <urdf_world/types.h>

#endif // URDFDOM_TYPEDEF_SHARED_PTR

#endif // ifndef __se3_parsers_urdf_types_hpp__
