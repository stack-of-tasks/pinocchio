//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_math_tensor_hpp__
#define __pinocchio_math_tensor_hpp__

#include "pinocchio/fwd.hpp"

#ifdef PINOCCHIO_WITH_CXX11_SUPPORT
  #include <unsupported/Eigen/CXX11/Tensor>
#endif

#if !EIGEN_VERSION_AT_LEAST(3,2,90)
  #define EIGEN_DEVICE_FUNC
#endif

namespace pinocchio
{

#ifndef PINOCCHIO_WITH_CXX11_SUPPORT

  // Mimic the Eigen::Tensor module only available for C++11 and more
  template<typename Scalar_, int NumIndices_, int Options_ = 0, typename IndexType = Eigen::DenseIndex>
  struct Tensor
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef Scalar_ Scalar;
    enum
    {
      Options = Options_,
      NumIndices = NumIndices_
    };
    typedef IndexType Index;
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index rank() const
    {
      return NumIndices;
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index dimension(std::size_t n) const
    {
      assert(n <= NumIndices && "n is larger than the dimension of the tensor.");
      return m_dimensions[n];
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Index size() const
    {
      return m_storage.size();
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Scalar *data()
    {
      return m_storage.data();
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Scalar *data() const 
    {
      return m_storage.data();
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE Tensor& setZero()
    {
      return setConstant(Scalar(0));
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE Tensor& setConstant(const Scalar & val)
    {
      m_storage.setConstant(val);
      return *this;
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE Tensor& setRandom()
    {
      m_storage.setRandom();
      return *this;
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE explicit Tensor(Index dim1)
    : m_storage(dim1)
    {
      m_dimensions[0] = dim1;
      EIGEN_STATIC_ASSERT(1 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor(Index dim1, Index dim2)
    : m_storage(dim1*dim2)
    {
      m_dimensions[0] = dim1;
      m_dimensions[1] = dim2;
      EIGEN_STATIC_ASSERT(2 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor(Index dim1, Index dim2, Index dim3)
    : m_storage(dim1*dim2*dim3)
    {
      m_dimensions[0] = dim1;
      m_dimensions[1] = dim2;
      m_dimensions[2] = dim3;
      EIGEN_STATIC_ASSERT(3 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor(Index dim1, Index dim2, Index dim3, Index dim4)
    : m_storage(dim1*dim2*dim3*dim4)
    {
      m_dimensions[0] = dim1;
      m_dimensions[1] = dim2;
      m_dimensions[2] = dim3;
      m_dimensions[3] = dim4;
      EIGEN_STATIC_ASSERT(4 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
    }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor(Index dim1, Index dim2, Index dim3, Index dim4, Index dim5)
    : m_storage(dim1*dim2*dim3*dim4*dim5)
    {
      m_dimensions[0] = dim1;
      m_dimensions[1] = dim2;
      m_dimensions[2] = dim3;
      m_dimensions[3] = dim4;
      m_dimensions[4] = dim5;
      EIGEN_STATIC_ASSERT(5 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i0) const
    {
      EIGEN_STATIC_ASSERT(1 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
      return m_storage.coeff(i0);
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i0, Index i1) const
    {
      EIGEN_STATIC_ASSERT(2 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
      return m_storage.coeff(i0 + i1 * m_dimensions[0]);
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i0, Index i1, Index i2) const
    {
      EIGEN_STATIC_ASSERT(3 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
      return m_storage.coeff(i0 + i1 * m_dimensions[0] + i2 * m_dimensions[1] * m_dimensions[0]);
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i0, Index i1, Index i2, Index i3) const
    {
      EIGEN_STATIC_ASSERT(4 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
      return coeff(i0 + i1 * m_dimensions[0] + i2 * m_dimensions[1] * m_dimensions[0] + i3 * m_dimensions[2] * m_dimensions[1] * m_dimensions[0]);
    }
    
    EIGEN_DEVICE_FUNC
    EIGEN_STRONG_INLINE const Scalar& operator()(Index i0, Index i1, Index i2, Index i3, Index i4) const
    {
      EIGEN_STATIC_ASSERT(5 == NumIndices, YOU_MADE_A_PROGRAMMING_MISTAKE)
      return coeff(i0 + i1 * m_dimensions[0] + i2 * m_dimensions[1] * m_dimensions[0] + i3 * m_dimensions[2] * m_dimensions[1] * m_dimensions[0] + i4 * m_dimensions[3] * m_dimensions[2] * m_dimensions[1] * m_dimensions[0]);
    }
    
  protected:
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> StorageType;
    StorageType m_storage;
    
    Index m_dimensions[NumIndices];
    
  };

#else

  // Use the default Eigen::Tensor module
  template<typename Scalar_, int NumIndices_, int Options_ = 0, typename IndexType = Eigen::DenseIndex>
  using Tensor = Eigen::Tensor<Scalar_,NumIndices_,Options_,IndexType>;

#endif // ifndef PINOCCHIO_WITH_CXX11_SUPPORT

}

#if !EIGEN_VERSION_AT_LEAST(3,2,90)
  #undef EIGEN_DEVICE_FUNC
#endif

#endif // ifndef __pinocchio_math_tensor_hpp__
