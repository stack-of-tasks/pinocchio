//
// Copyright (c) 2016-2024 CNRS INRIA
//

#ifndef __pinocchio_container_aligned_vector_hpp__
#define __pinocchio_container_aligned_vector_hpp__

#include <vector>
#include <Eigen/StdVector>

#define PINOCCHIO_ALIGNED_STD_VECTOR(Type) ::pinocchio::container::aligned_vector<Type>

#define PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(T) ::pinocchio::container::aligned_vector<T>

namespace pinocchio
{
  namespace container
  {

    template<typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

  } // namespace container

} // namespace pinocchio

#endif // ifndef __pinocchio_container_aligned_vector_hpp__
