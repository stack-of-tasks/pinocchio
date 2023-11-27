//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_math_tensor_hpp__
#define __pinocchio_math_tensor_hpp__

#include "pinocchio/fwd.hpp"

#if !EIGEN_VERSION_AT_LEAST(3,2,90)
  #define EIGEN_DEVICE_FUNC
#endif

#ifndef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
  #if (__cplusplus <= 199711L && EIGEN_COMP_MSVC < 1900) || defined(__CUDACC__) || defined(EIGEN_AVOID_STL_ARRAY)
    namespace Eigen {
      template <typename T, std::size_t n>
      struct array
      {
        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE T& operator[] (size_t index)
        { return values[index]; }
        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE const T& operator[] (size_t index) const
        { return values[index]; }

        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE T& front() { return values[0]; }
        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE const T& front() const { return values[0]; }

        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE T& back() { return values[n-1]; }
        EIGEN_DEVICE_FUNC
        EIGEN_STRONG_INLINE const T& back() const { return values[n-1]; }

        EIGEN_DEVICE_FUNC EIGEN_ALWAYS_INLINE
        static std::size_t size() { return n; }

        T values[n];
      };
    
      template<class T, std::size_t n>
      EIGEN_DEVICE_FUNC bool operator==(const array<T,n> & lhs, const array<T,n> & rhs)
      {
        for (std::size_t i = 0; i < n; ++i) {
          if (lhs[i] != rhs[i]) {
            return false;
          }
        }
        return true;
      }
        
      template<class T, std::size_t n>
      EIGEN_DEVICE_FUNC bool operator!=(const array<T,n> & lhs, const array<T,n> & rhs)
      {
        return !(lhs == rhs);
      }
    } // namespace Eigen
  #else
    #include <array>
    namespace Eigen {
      template <typename T, std::size_t N> using array = std::array<T, N>;
    } // namespace Eigen
  #endif
#endif

namespace pinocchio
{

#ifndef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE

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
    typedef Eigen::array<Index,NumIndices_> Dimensions;
    
    inline Tensor& base()             { return *this; }
    inline const Tensor& base() const { return *this; }
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
    Dimensions& dimensions() { return m_dimensions; }
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
    const Dimensions& dimensions() const { return m_dimensions; }
    
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
    
    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor()
    : m_storage()
    {
    }

    EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE Tensor(const Tensor & other)
    : m_storage(other.m_storage)
    , m_dimensions(other.m_dimensions)
    {
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
    
    EIGEN_DEVICE_FUNC
    void resize(const Eigen::array<Index,NumIndices> & dimensions)
    {
      size_t i;
      Index size = Index(1);
      for(i = 0; i < NumIndices; i++)
      {
        Eigen::internal::check_rows_cols_for_overflow<Eigen::Dynamic>::run(size, dimensions[i]);
        size *= dimensions[i];
      }
      
      for(i = 0; i < NumIndices; i++)
        m_dimensions[i] = dimensions[i];
      
      bool size_changed = size != this->size();
      if(size_changed) m_storage.resize(size);
      
#ifdef EIGEN_INITIALIZE_COEFFS
        if(size_changed)
        {
#if defined(EIGEN_INITIALIZE_MATRICES_BY_ZERO)
          m_storage.fill(Scalar(0));
#elif defined(EIGEN_INITIALIZE_MATRICES_BY_NAN)
          m_storage.fill(std::numeric_limits<Scalar>::quiet_NaN());
#endif
        }
#endif
    }
    
    EIGEN_DEVICE_FUNC bool operator==(const Tensor & other) const
    {
      return m_storage == other.m_storage;
    }
    
    EIGEN_DEVICE_FUNC bool operator!=(const Tensor & other) const
    {
      return m_storage != other.m_storage;
    }
    
  protected:
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> StorageType;
    StorageType m_storage;
    
    Dimensions m_dimensions;
    
  };

#else

  // Use the default Eigen::Tensor module
  template<typename Scalar_, int NumIndices_, int Options_ = 0, typename IndexType = Eigen::DenseIndex>
  using Tensor = Eigen::Tensor<Scalar_,NumIndices_,Options_,IndexType>;

#endif // ifndef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE

}

#if !EIGEN_VERSION_AT_LEAST(3,2,90)
  #undef EIGEN_DEVICE_FUNC
#endif

#endif // ifndef __pinocchio_math_tensor_hpp__
