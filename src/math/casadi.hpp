//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_math_casadi_hpp__
#define __pinocchio_math_casadi_hpp__

#include <casadi/casadi.hpp>
// #include <Eigen/Core>
#include <Eigen/Dense>

// Copy casadi matrix to Eigen matrix
template <typename MT>
inline void copy(casadi::SX const& src, Eigen::MatrixBase<MT>& dst)
{
  Eigen::Index const m = src.size1();
  Eigen::Index const n = src.size2();

  dst.resize(m, n);

  for (Eigen::Index i = 0; i < m; ++i)
    for (Eigen::Index j = 0; j < n; ++j)
      dst(i, j) = src(i, j);
}


// Copy Eigen matrix to casadi matrix
template <typename MT>
inline void copy(Eigen::MatrixBase<MT> const& src, casadi::SX& dst)
{
  Eigen::Index const m = src.rows();
  Eigen::Index const n = src.cols();

  dst.resize(m, n);

  for (Eigen::Index i = 0; i < m; ++i)
    for (Eigen::Index j = 0; j < n; ++j)
      dst(i, j) = src(i, j);
}


// Make an Eigen matrix consisting of pure casadi symbolics
template <typename MT>
inline void sym(Eigen::MatrixBase<MT>& eig_mat, std::string const& name)
{
  for (Eigen::Index i = 0; i < eig_mat.rows(); ++i)
    for (Eigen::Index j = 0; j < eig_mat.cols(); ++j)
      eig_mat(i, j) = casadi::SX::sym(name + "_" + std::to_string(i) + "_" + std::to_string(j));
}


namespace Eigen
{
  namespace internal
  {
    // Specialization of Eigen::internal::cast_impl for Casadi input types
    template<typename Scalar>
    struct cast_impl<casadi::SX,Scalar>
    {
#if EIGEN_VERSION_AT_LEAST(3,2,90)
      EIGEN_DEVICE_FUNC
#endif
      static inline Scalar run(const casadi::SX & x)
      {
        return static_cast<Scalar>(x);
      }
    };
  }
}

namespace Eigen
{
  /// @brief Eigen::NumTraits<> specialization for casadi::SX
  ///
  template<> struct NumTraits<casadi::SX>
  {
    using Real = casadi::SX;
    using NonInteger = casadi::SX;
    using Literal = casadi::SX;
    using Nested = casadi::SX;
    
    static bool constexpr IsComplex = false;
    static bool constexpr IsInteger = false;
    static int constexpr ReadCost = 1;
    static int constexpr AddCost = 1;
    static int constexpr MulCost = 1;
    static bool constexpr IsSigned = true;
    static bool constexpr RequireInitialization = true;
    
    static double epsilon()
    {
      return std::numeric_limits<double>::epsilon();
    }
    
    static double dummy_precision()
    {
      return NumTraits<double>::dummy_precision();
    }
    
    static double hightest()
    {
      return std::numeric_limits<double>::max();
    }
    
    static double lowest()
    {
      return std::numeric_limits<double>::min();
    }
    
    static int digits10()
    {
      return std::numeric_limits<double>::digits10;
    }
  };


  namespace internal
  {
    /// @brief LLT_Traits for casadi::SX-valued matrices
    ///
    /// LLT specialization for matrices of casadi::SX containts in m_matrix the L such as LL'=A,
    /// regardless of the value of UpLo.
    /// Therefore we define two identical specializations of LLT_Traits for the two values of UpLo.
    template<int M, int N, int Options>
    struct LLT_Traits<Matrix<casadi::SX, M, N, Options>, Lower>
    {
      using MatrixType = Matrix<casadi::SX, M, N, Options>;
      typedef const TriangularView<const MatrixType, Lower> MatrixL;
      typedef const TriangularView<const typename MatrixType::AdjointReturnType, Upper> MatrixU;
      static inline MatrixL getL(const MatrixType& m) { return MatrixL(m); }
      static inline MatrixU getU(const MatrixType& m) { return MatrixU(m.adjoint()); }
      static bool inplace_decomposition(MatrixType& m)
      { return llt_inplace<typename MatrixType::Scalar, Lower>::blocked(m)==-1; }
    };


    /// @brief LLT_Traits for casadi::SX-valued matrices
    ///
    /// LLT specialization for matrices of casadi::SX containts in m_matrix the L such as LL'=A,
    /// regardless of the value of UpLo.
    /// Therefore we define two identical specializations of LLT_Traits for the two values of UpLo.
    template<int M, int N, int Options>
    struct LLT_Traits<Matrix<casadi::SX, M, N, Options>, Upper>
    {
      using MatrixType = Matrix<casadi::SX, M, N, Options>;
      typedef const TriangularView<const typename MatrixType::AdjointReturnType, Lower> MatrixL;
      typedef const TriangularView<const MatrixType, Upper> MatrixU;
      static inline MatrixL getL(const MatrixType& m) { return MatrixL(m); }
      static inline MatrixU getU(const MatrixType& m) { return MatrixU(m.adjoint()); }
      static bool inplace_decomposition(MatrixType& m)
      { return llt_inplace<typename MatrixType::Scalar, Upper>::blocked(m)==-1; }
    };
  }


  /// @brief LLT template specialization supporting casadi::SX
  ///
  template<int M, int N, int Options, int _UpLo>
  class LLT<Matrix<casadi::SX, M, N, Options>, _UpLo>
  {
    public:
      typedef Matrix<casadi::SX, M, N, Options> MatrixType;
      enum {
        RowsAtCompileTime = MatrixType::RowsAtCompileTime,
        ColsAtCompileTime = MatrixType::ColsAtCompileTime,
        MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
      };
      typedef typename MatrixType::Scalar Scalar;
      typedef typename NumTraits<typename MatrixType::Scalar>::Real RealScalar;
      typedef Eigen::Index Index; ///< \deprecated since Eigen 3.3
      typedef typename MatrixType::StorageIndex StorageIndex;

      enum {
        PacketSize = internal::packet_traits<Scalar>::size,
        AlignmentMask = int(PacketSize)-1,
        UpLo = _UpLo
      };

      typedef internal::LLT_Traits<MatrixType,UpLo> Traits;

      /**
        * \brief Default Constructor.
        *
        * The default constructor is useful in cases in which the user intends to
        * perform decompositions via LLT::compute(const MatrixType&).
        */
      LLT() : m_matrix(), m_isInitialized(false) {}

      /** \brief Default Constructor with memory preallocation
        *
        * Like the default constructor but with preallocation of the internal data
        * according to the specified problem \a size.
        * \sa LLT()
        */
      explicit LLT(Index size) : m_matrix(size, size),
                      m_isInitialized(false) {}

      template<typename InputType>
      explicit LLT(const EigenBase<InputType>& matrix)
        : m_matrix(matrix.rows(), matrix.cols()),
          m_isInitialized(false)
      {
        compute(matrix.derived());
      }

      /** \brief Constructs a LDLT factorization from a given matrix
        *
        * This overloaded constructor is provided for \link InplaceDecomposition inplace decomposition \endlink when
        * \c MatrixType is a Eigen::Ref.
        *
        * \sa LLT(const EigenBase&)
        */
      template<typename InputType>
      explicit LLT(EigenBase<InputType>& matrix)
        : m_matrix(matrix.derived()),
          m_isInitialized(false)
      {
        compute(matrix.derived());
      }

      /** \returns a view of the upper triangular matrix U */
      inline typename Traits::MatrixU matrixU() const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        return Traits::getU(m_matrix);
      }

      /** \returns a view of the lower triangular matrix L */
      inline typename Traits::MatrixL matrixL() const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        return Traits::getL(m_matrix);
      }

      /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
        *
        * Since this LLT class assumes anyway that the matrix A is invertible, the solution
        * theoretically exists and is unique regardless of b.
        *
        * Example: \include LLT_solve.cpp
        * Output: \verbinclude LLT_solve.out
        *
        * \sa solveInPlace(), MatrixBase::llt(), SelfAdjointView::llt()
        */
      template<typename Rhs>
      inline const Solve<LLT, Rhs>
      solve(const MatrixBase<Rhs>& b) const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        eigen_assert(m_matrix.rows()==b.rows()
                  && "LLT::solve(): invalid number of rows of the right hand side matrix b");
        return Solve<LLT, Rhs>(*this, b.derived());
      }

      template<typename Derived>
      void solveInPlace(const MatrixBase<Derived> &bAndX) const;

      template<typename InputType>
      LLT& compute(const EigenBase<InputType>& matrix)
      {
        // Copy the triangular part of matrix that will be used for the decompositon to a self-adjoint m_matrix,
        // taking into account the UpLo flag.
        m_matrix = SelfAdjointView<InputType const, UpLo>(matrix);

        // The Eigen version of LLT::compute() does not compile for matrices of casadi::SX,
        // because operator>() for casadi::SX is not defined.
        // Therefore, we use casadi::chol() to do the decomposition, and then copy the result to an Eigen matrix of casadi::SX.

        // Copy m_matrix to a temporary casadi::SX matrix
        casadi::SX cs_matrix;
        copy(m_matrix, cs_matrix);

        // Let casadi do the decomposition.
        // The result needs to be transposed, because casadi returns an upper triangular R such that R'R = A
        // http://casadi.sourceforge.net/api/html/da/d45/classcasadi_1_1Matrix.html
        // and we need L such that LL' = A.
        cs_matrix = transpose(chol(cs_matrix));

        // Copy the result to m_matrix and set the isInitialized flag.
        copy(cs_matrix, m_matrix);
        m_isInitialized = true;

        return *this;
      }

      /** \returns an estimate of the reciprocal condition number of the matrix of
        *  which \c *this is the Cholesky decomposition.
        */
      RealScalar rcond() const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        eigen_assert(m_info == Success && "LLT failed because matrix appears to be negative");
        return internal::rcond_estimate_helper(m_l1_norm, *this);
      }

      /** \returns the LLT decomposition matrix
        *
        * TODO: document the storage layout
        */
      inline const MatrixType& matrixLLT() const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        return m_matrix;
      }

      MatrixType reconstructedMatrix() const;


      /** \brief Reports whether previous computation was successful.
        *
        * \returns \c Success if computation was succesful,
        *          \c NumericalIssue if the matrix.appears not to be positive definite.
        */
      ComputationInfo info() const
      {
        eigen_assert(m_isInitialized && "LLT is not initialized.");
        return m_info;
      }

      /** \returns the adjoint of \c *this, that is, a const reference to the decomposition itself as the underlying matrix is self-adjoint.
        *
        * This method is provided for compatibility with other matrix decompositions, thus enabling generic code such as:
        * \code x = decomposition.adjoint().solve(b) \endcode
        */
      const LLT& adjoint() const { return *this; };

      inline Index rows() const { return m_matrix.rows(); }
      inline Index cols() const { return m_matrix.cols(); }

      template<typename VectorType>
      LLT rankUpdate(const VectorType& vec, const RealScalar& sigma = 1);

      #ifndef EIGEN_PARSED_BY_DOXYGEN
      template<typename RhsType, typename DstType>
      EIGEN_DEVICE_FUNC
      void _solve_impl(const RhsType &rhs, DstType &dst) const;
      #endif

    protected:

      static void check_template_parameters()
      {
        EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar);
      }

      /** \internal
        * Used to compute and store L
        * The strict upper part is not used and even not initialized.
        */
      MatrixType m_matrix;
      RealScalar m_l1_norm;
      bool m_isInitialized;
      ComputationInfo m_info;
  };
}

#endif // #ifndef __pinocchio_math_casadi_hpp__
