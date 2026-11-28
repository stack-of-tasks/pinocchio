//
// Copyright (c) 2020-2025 INRIA
//

#pragma once

#include <string>
#include <sstream>

#include "pinocchio/math.hpp"

namespace pinocchio
{
  namespace python
  {
    template<typename MatrixLike>
    std::string getEigenTypeName()
    {
      std::stringstream ss;
      if (::pinocchio::helper::is_eigen_ref<MatrixLike>::value)
      {
        ss << "Ref";
      }

      if (MatrixLike::IsRowMajor)
        ss << "Row";

      if (MatrixLike::IsVectorAtCompileTime)
      {
        ss << "Vector";
      }
      else
      {
        ss << "Matrix";
      }

      if (MatrixLike::SizeAtCompileTime == Eigen::Dynamic)
      {
        ss << "X";
      }
      else
      {
        ss << std::to_string(MatrixLike::SizeAtCompileTime);
      }

      return ss.str();
    }

    /// \brief Returns a row-major copy of a column-major tensor.
    ///
    /// eigenpy converts a tensor to a C-ordered numpy array by copying its coefficients in storage
    /// order. Returning a row-major copy thus gives a numpy array A with the same indexing as the
    /// C++ tensor, i.e. A[i, j, k] = tensor(i, j, k).
    template<typename TensorType>
    Eigen::Tensor<
      typename TensorType::Scalar,
      TensorType::NumIndices,
      Eigen::RowMajor,
      typename TensorType::Index>
    toRowMajorTensor(const TensorType & tensor)
    {
      static_assert(
        !(TensorType::Options & Eigen::RowMajor), "The input tensor must be column-major.");
      typedef typename TensorType::Index Index;
      enum
      {
        NumIndices = TensorType::NumIndices
      };

      // swap_layout reverses the order of the dimensions, shuffle restores it.
      Eigen::array<Index, NumIndices> reverse_dimensions;
      for (int k = 0; k < NumIndices; ++k)
        reverse_dimensions[k] = NumIndices - 1 - k;
      return tensor.swap_layout().shuffle(reverse_dimensions);
    }
  } // namespace python
} // namespace pinocchio
