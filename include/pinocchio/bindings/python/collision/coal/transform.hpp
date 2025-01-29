//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_python_collision_fcl_transform_hpp__
#define __pinocchio_python_collision_fcl_transform_hpp__

#include "pinocchio/spatial/se3.hpp"
#include <coal/math/transform.h>

namespace boost
{
  namespace python
  {
    namespace converter
    {

      template<typename Scalar, int Options>
      struct implicit<::coal::Transform3f, ::pinocchio::SE3Tpl<Scalar, Options>>
      {
        typedef ::coal::Transform3f Source;
        typedef ::pinocchio::SE3Tpl<Scalar, Options> Target;

        static void * convertible(PyObject * obj)
        {
          // Find a converter which can produce a Source instance from
          // obj. The user has told us that Source can be converted to
          // Target, and instantiating construct() below, ensures that
          // at compile-time.
          return implicit_rvalue_convertible_from_python(obj, registered<Source>::converters) ? obj
                                                                                              : 0;
        }

        static void construct(PyObject * obj, rvalue_from_python_stage1_data * data)
        {
          void * storage =
            reinterpret_cast<rvalue_from_python_storage<Target> *>(reinterpret_cast<void *>(data))
              ->storage.bytes;

          arg_from_python<Source> get_source(obj);
          bool convertible = get_source.convertible();
          BOOST_VERIFY(convertible);

          const Source & t = get_source();
          new (storage) Target(t.getRotation(), t.getTranslation());

          // record successful construction
          data->convertible = storage;
        }
      };

      template<typename Scalar, int Options>
      struct implicit<::pinocchio::SE3Tpl<Scalar, Options>, ::coal::Transform3f>
      {
        typedef ::pinocchio::SE3Tpl<Scalar, Options> Source;
        typedef ::coal::Transform3f Target;

        static void * convertible(PyObject * obj)
        {
          // Find a converter which can produce a Source instance from
          // obj. The user has told us that Source can be converted to
          // Target, and instantiating construct() below, ensures that
          // at compile-time.
          return implicit_rvalue_convertible_from_python(obj, registered<Source>::converters) ? obj
                                                                                              : 0;
        }

        static void construct(PyObject * obj, rvalue_from_python_stage1_data * data)
        {
          void * storage =
            reinterpret_cast<rvalue_from_python_storage<Target> *>(reinterpret_cast<void *>(data))
              ->storage.bytes;

          arg_from_python<Source> get_source(obj);
          bool convertible = get_source.convertible();
          BOOST_VERIFY(convertible);

          const Source & M = get_source();
          new (storage) Target(M.rotation(), M.translation());

          // record successful construction
          data->convertible = storage;
        }
      };

    } // namespace converter
  } // namespace python
} // namespace boost

#endif // ifndef __pinocchio_python_collision_fcl_transform_hpp__
