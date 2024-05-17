//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_math_tridiagonal_matrix_hpp__
#define __pinocchio_math_tridiagonal_matrix_hpp__

#include "pinocchio/fwd.hpp"
#include <Eigen/Dense>

namespace pinocchio
{
  template<typename Scalar, int Options = 0>
  struct TridiagonalSymmetricMatrixTpl;

  template<typename _Scalar, int _Options>
  struct traits<TridiagonalSymmetricMatrixTpl<_Scalar, _Options>>
  {
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> PlainMatrixType;
  };

  template<typename TridiagonalSymmetricMatrix, typename MatrixDerived>
  struct TridiagonalSymmetricMatrixApplyOnTheRightReturnType;

  template<typename TridiagonalSymmetricMatrix, typename MatrixDerived>
  struct traits<
    TridiagonalSymmetricMatrixApplyOnTheRightReturnType<TridiagonalSymmetricMatrix, MatrixDerived>>
  : public traits<TridiagonalSymmetricMatrix>
  {
  };

  template<typename MatrixDerived, typename TridiagonalSymmetricMatrix>
  struct TridiagonalSymmetricMatrixApplyOnTheLeftReturnType;

  template<typename MatrixDerived, typename TridiagonalSymmetricMatrix>
  struct traits<
    TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<MatrixDerived, TridiagonalSymmetricMatrix>>
  : public traits<TridiagonalSymmetricMatrix>
  {
  };

  template<typename TridiagonalSymmetricMatrix>
  struct TridiagonalSymmetricMatrixInverse;

  template<typename TridiagonalSymmetricMatrix>
  struct traits<TridiagonalSymmetricMatrixInverse<TridiagonalSymmetricMatrix>>
  : public traits<TridiagonalSymmetricMatrix>
  {
  };

  template<typename TridiagonalSymmetricMatrixInverse, typename MatrixDerived>
  struct TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType;

  template<typename TridiagonalSymmetricMatrixInverse, typename MatrixDerived>
  struct traits<TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<
    TridiagonalSymmetricMatrixInverse,
    MatrixDerived>> : public traits<TridiagonalSymmetricMatrixInverse>
  {
  };
} // namespace pinocchio

namespace Eigen
{
  namespace internal
  {

    template<typename Scalar, int Options>
    struct traits<pinocchio::TridiagonalSymmetricMatrixTpl<Scalar, Options>>
    : public traits<typename pinocchio::traits<
        pinocchio::TridiagonalSymmetricMatrixTpl<Scalar, Options>>::PlainMatrixType>
    {
      typedef pinocchio::traits<pinocchio::TridiagonalSymmetricMatrixTpl<Scalar, Options>> Base;
      typedef typename Base::PlainMatrixType ReturnType;
      enum
      {
        Flags = 0
      };
    };

    template<typename TridiagonalSymmetricMatrix, typename MatrixDerived>
    struct traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheRightReturnType<
      TridiagonalSymmetricMatrix,
      MatrixDerived>>
    : public traits<
        typename pinocchio::traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheRightReturnType<
          TridiagonalSymmetricMatrix,
          MatrixDerived>>::PlainMatrixType>
    {
      typedef pinocchio::traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheRightReturnType<
        TridiagonalSymmetricMatrix,
        MatrixDerived>>
        Base;
      typedef typename Base::PlainMatrixType ReturnType;
      enum
      {
        Flags = 0
      };
    };

    template<typename MatrixDerived, typename TridiagonalSymmetricMatrix>
    struct traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<
      MatrixDerived,
      TridiagonalSymmetricMatrix>>
    : public traits<
        typename pinocchio::traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<
          MatrixDerived,
          TridiagonalSymmetricMatrix>>::PlainMatrixType>
    {
      typedef pinocchio::traits<pinocchio::TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<
        MatrixDerived,
        TridiagonalSymmetricMatrix>>
        Base;
      typedef typename Base::PlainMatrixType ReturnType;
      enum
      {
        Flags = 0
      };
    };

    template<typename TridiagonalSymmetricMatrix>
    struct traits<pinocchio::TridiagonalSymmetricMatrixInverse<TridiagonalSymmetricMatrix>>
    : public traits<TridiagonalSymmetricMatrix>
    {
    };

    template<typename TridiagonalSymmetricMatrixInverse, typename MatrixDerived>
    struct traits<pinocchio::TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<
      TridiagonalSymmetricMatrixInverse,
      MatrixDerived>>
    : public traits<typename pinocchio::traits<
        pinocchio::TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<
          TridiagonalSymmetricMatrixInverse,
          MatrixDerived>>::PlainMatrixType>
    {
      typedef pinocchio::traits<
        pinocchio::TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<
          TridiagonalSymmetricMatrixInverse,
          MatrixDerived>>
        Base;
      typedef typename Base::PlainMatrixType ReturnType;
      enum
      {
        Flags = 0
      };
    };

  } // namespace internal
} // namespace Eigen

namespace pinocchio
{

  /// \brief Dynamic size Tridiagonal symmetric matrix type
  ///  This class is in practice used in Lanczos decomposition
  template<typename _Scalar, int _Options>
  struct TridiagonalSymmetricMatrixTpl
  : public Eigen::ReturnByValue<TridiagonalSymmetricMatrixTpl<_Scalar, _Options>>
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef TridiagonalSymmetricMatrixTpl Self;
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> CoeffVectorType;
    typedef typename traits<Self>::PlainMatrixType PlainMatrixType;

    /// \brief Default constructor from a given size
    explicit TridiagonalSymmetricMatrixTpl(const Eigen::DenseIndex size)
    : m_size(size)
    , m_diagonal(size)
    , m_sub_diagonal(size - 1)
    {
      assert(size > 0 && "size should be greater than 0.");
    }

    bool operator==(const TridiagonalSymmetricMatrixTpl & other) const
    {
      if (this == &other)
        return true;
      return diagonal() == other.diagonal() && subDiagonal() == other.subDiagonal();
    }

    bool operator!=(const TridiagonalSymmetricMatrixTpl & other) const
    {
      return !(*this == other);
    }

    TridiagonalSymmetricMatrixInverse<Self> inverse() const
    {
      return TridiagonalSymmetricMatrixInverse<Self>(*this);
    }

    CoeffVectorType & diagonal()
    {
      return m_diagonal;
    }
    const CoeffVectorType & diagonal() const
    {
      return m_diagonal;
    }
    CoeffVectorType & subDiagonal()
    {
      return m_sub_diagonal;
    }
    const CoeffVectorType & subDiagonal() const
    {
      return m_sub_diagonal;
    }

    void setIdentity()
    {
      diagonal().setOnes();
      subDiagonal().setZero();
    }

    bool isIdentity(const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return subDiagonal().isZero(prec) && diagonal().isOnes(prec);
    }

    void setZero()
    {
      diagonal().setZero();
      subDiagonal().setZero();
    }

    bool isZero(const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return subDiagonal().isZero(prec) && diagonal().isZero(prec);
    }

    void setRandom()
    {
      diagonal().setRandom();
      subDiagonal().setRandom();
    }

    bool isDiagonal(const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return subDiagonal().isZero(prec);
    }

    template<typename VectorCoeffType>
    void setDiagonal(const Eigen::MatrixBase<VectorCoeffType> & diagonal_coefficients)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(diagonal_coefficients.size(), cols());
      static_assert(
        VectorCoeffType::IsVectorAtCompileTime,
        "VectorCoeffType should be a vector type at compile time");

      diagonal() = diagonal_coefficients;
      subDiagonal().setZero();
    }

    EIGEN_CONSTEXPR Eigen::Index rows() const EIGEN_NOEXCEPT
    {
      return m_size;
    }
    EIGEN_CONSTEXPR Eigen::Index cols() const EIGEN_NOEXCEPT
    {
      return m_size;
    }

    PlainMatrixType matrix() const
    {
      return PlainMatrixType(*this);
    }

    template<typename ResultType>
    inline void evalTo(ResultType & result) const
    {
      result.setZero();
      result.template diagonal<1>() = subDiagonal().conjugate();
      result.diagonal() = diagonal();
      result.template diagonal<-1>() = subDiagonal();
    }

    template<typename MatrixDerived>
    TridiagonalSymmetricMatrixApplyOnTheRightReturnType<Self, MatrixDerived>
    applyOnTheRight(const Eigen::MatrixBase<MatrixDerived> & mat) const
    {
      typedef TridiagonalSymmetricMatrixApplyOnTheRightReturnType<Self, MatrixDerived> ReturnType;
      return ReturnType(*this, mat.derived());
    }

    template<typename MatrixDerived>
    TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<MatrixDerived, Self>
    applyOnTheLeft(const Eigen::MatrixBase<MatrixDerived> & mat) const
    {
      typedef TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<MatrixDerived, Self> ReturnType;
      return ReturnType(mat.derived(), *this);
    }

    template<typename MatrixDerived>
    inline TridiagonalSymmetricMatrixApplyOnTheRightReturnType<Self, MatrixDerived>
    operator*(const Eigen::MatrixBase<MatrixDerived> & mat) const
    {
      return this->applyOnTheRight(mat.derived());
    }

  protected:
    Eigen::DenseIndex m_size;
    CoeffVectorType m_diagonal;
    CoeffVectorType m_sub_diagonal;
  };

  template<typename LhsMatrixType, typename S, int O>
  TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<
    LhsMatrixType,
    TridiagonalSymmetricMatrixTpl<S, O>>
  operator*(
    const Eigen::MatrixBase<LhsMatrixType> & lhs, const TridiagonalSymmetricMatrixTpl<S, O> & rhs)
  {
    return rhs.applyOnTheLeft(lhs);
  }

  template<typename TridiagonalSymmetricMatrix, typename RhsMatrixType>
  struct TridiagonalSymmetricMatrixApplyOnTheRightReturnType
  : public Eigen::ReturnByValue<TridiagonalSymmetricMatrixApplyOnTheRightReturnType<
      TridiagonalSymmetricMatrix,
      RhsMatrixType>>
  {
    typedef TridiagonalSymmetricMatrixApplyOnTheRightReturnType Self;
    typedef typename traits<Self>::PlainMatrixType PlainMatrixType;

    TridiagonalSymmetricMatrixApplyOnTheRightReturnType(
      const TridiagonalSymmetricMatrix & lhs, const RhsMatrixType & rhs)
    : m_lhs(lhs)
    , m_rhs(rhs)
    {
    }

    template<typename ResultType>
    inline void evalTo(ResultType & result) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.rows(), rows());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.cols(), cols());

      assert(cols() >= 1);
      assert(rows() >= 1);

      const Eigen::DenseIndex reduced_size = cols() - 1;
      // Main diagonal
      result.noalias() = m_lhs.diagonal().asDiagonal() * m_rhs;
      // Upper diagonal
      result.topRows(reduced_size).noalias() +=
        m_lhs.subDiagonal().conjugate().asDiagonal() * m_rhs.bottomRows(reduced_size);
      // Sub diagonal
      result.bottomRows(reduced_size).noalias() +=
        m_lhs.subDiagonal().asDiagonal() * m_rhs.topRows(reduced_size);
    }

    EIGEN_CONSTEXPR Eigen::Index rows() const EIGEN_NOEXCEPT
    {
      return m_lhs.rows();
    }
    EIGEN_CONSTEXPR Eigen::Index cols() const EIGEN_NOEXCEPT
    {
      return m_rhs.cols();
    }

  protected:
    const TridiagonalSymmetricMatrix & m_lhs;
    const RhsMatrixType & m_rhs;
  };

  template<typename LhsMatrixType, typename TridiagonalSymmetricMatrix>
  struct TridiagonalSymmetricMatrixApplyOnTheLeftReturnType
  : public Eigen::ReturnByValue<
      TridiagonalSymmetricMatrixApplyOnTheLeftReturnType<LhsMatrixType, TridiagonalSymmetricMatrix>>
  {
    typedef TridiagonalSymmetricMatrixApplyOnTheLeftReturnType Self;
    typedef typename traits<Self>::PlainMatrixType PlainMatrixType;

    TridiagonalSymmetricMatrixApplyOnTheLeftReturnType(
      const LhsMatrixType & lhs, const TridiagonalSymmetricMatrix & rhs)
    : m_lhs(lhs)
    , m_rhs(rhs)
    {
    }

    template<typename ResultType>
    inline void evalTo(ResultType & result) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.rows(), rows());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.cols(), cols());

      assert(cols() >= 1);
      assert(rows() >= 1);

      const Eigen::DenseIndex reduced_size = cols() - 1;
      // Main diagonal
      result.noalias() = m_lhs * m_rhs.diagonal().asDiagonal();
      // Upper diagonal
      result.rightCols(reduced_size).noalias() +=
        m_lhs.leftCols(reduced_size) * m_rhs.subDiagonal().conjugate().asDiagonal();
      // Sub diagonal
      result.leftCols(reduced_size).noalias() +=
        m_lhs.rightCols(reduced_size) * m_rhs.subDiagonal().asDiagonal();
    }

    EIGEN_CONSTEXPR Eigen::Index rows() const EIGEN_NOEXCEPT
    {
      return m_lhs.rows();
    }
    EIGEN_CONSTEXPR Eigen::Index cols() const EIGEN_NOEXCEPT
    {
      return m_rhs.cols();
    }

  protected:
    const LhsMatrixType & m_lhs;
    const TridiagonalSymmetricMatrix & m_rhs;
  };

  template<typename _TridiagonalSymmetricMatrix>
  struct TridiagonalSymmetricMatrixInverse
  : public Eigen::ReturnByValue<TridiagonalSymmetricMatrixInverse<_TridiagonalSymmetricMatrix>>
  {
    typedef _TridiagonalSymmetricMatrix TridiagonalSymmetricMatrix;
    typedef TridiagonalSymmetricMatrixInverse Self;
    typedef typename TridiagonalSymmetricMatrix::Scalar Scalar;
    enum
    {
      Options = TridiagonalSymmetricMatrix::Options
    };

    typedef typename TridiagonalSymmetricMatrix::CoeffVectorType CoeffVectorType;
    typedef typename traits<Self>::PlainMatrixType PlainMatrixType;

    explicit TridiagonalSymmetricMatrixInverse(
      const TridiagonalSymmetricMatrix & tridiagonal_matrix)
    : tridiagonal_matrix(tridiagonal_matrix)
    , m_size(tridiagonal_matrix.rows())
    , m_diagonal(m_size)
    , m_sub_diagonal(m_size - 1)
    {
      eval();
    }

    const TridiagonalSymmetricMatrix & inverse() const
    {
      return tridiagonal_matrix;
    }

    template<typename MatrixDerived>
    TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<Self, MatrixDerived>
    applyOnTheRight(const Eigen::MatrixBase<MatrixDerived> & mat) const
    {
      typedef TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<Self, MatrixDerived>
        ReturnType;
      return ReturnType(*this, mat.derived());
    }

    template<typename MatrixDerived>
    inline TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<Self, MatrixDerived>
    operator*(const Eigen::MatrixBase<MatrixDerived> & mat) const
    {
      return this->applyOnTheRight(mat.derived());
    }

    template<typename ResultType>
    inline void evalTo(ResultType & result) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.rows(), rows());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.cols(), cols());

      assert(cols() >= 1);
      assert(rows() >= 1);

      const auto & b = m_diagonal;
      const auto & c = tridiagonal_matrix.subDiagonal();
      const auto & w = m_sub_diagonal;

      // Forward sweep
      result.setIdentity();
      for (Eigen::DenseIndex i = 1; i < m_size; ++i)
      {
        result.row(i).head(i) -= w[i - 1] * result.row(i - 1).head(i);
      }

      // Backward sweep
      result.row(m_size - 1) /= b[m_size - 1];
      for (Eigen::DenseIndex i = m_size - 2; i >= 0; --i)
      {
        result.row(i) -= c[i] * result.row(i + 1);
        result.row(i) /= b[i];
      }
    }

    EIGEN_CONSTEXPR Eigen::Index rows() const EIGEN_NOEXCEPT
    {
      return m_size;
    }
    EIGEN_CONSTEXPR Eigen::Index cols() const EIGEN_NOEXCEPT
    {
      return m_size;
    }

  protected:
    template<typename T, typename MatrixDerived>
    friend struct TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType;

    /// \brief Forward sweep of https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
    void eval()
    {
      m_diagonal = tridiagonal_matrix.diagonal();
      m_sub_diagonal = tridiagonal_matrix.subDiagonal();
      auto & w = m_sub_diagonal;
      auto & b = m_diagonal;
      const auto & c = tridiagonal_matrix.subDiagonal();
      for (Eigen::DenseIndex i = 1; i < m_size; ++i)
      {
        w.coeffRef(i - 1) /= b[i - 1];
        m_diagonal.coeffRef(i) -= w[i - 1] * c[i - 1];
      }
    }

    const TridiagonalSymmetricMatrix & tridiagonal_matrix;
    Eigen::DenseIndex m_size;
    CoeffVectorType m_diagonal;
    CoeffVectorType m_sub_diagonal;
  };

  template<typename TridiagonalSymmetricMatrixInverse, typename RhsMatrixType>
  struct TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType
  : public Eigen::ReturnByValue<TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType<
      TridiagonalSymmetricMatrixInverse,
      RhsMatrixType>>
  {
    typedef TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType Self;
    typedef typename traits<Self>::PlainMatrixType PlainMatrixType;

    TridiagonalSymmetricMatrixInverseApplyOnTheRightReturnType(
      const TridiagonalSymmetricMatrixInverse & lhs, const RhsMatrixType & rhs)
    : m_lhs(lhs)
    , m_rhs(rhs)
    {
    }

    template<typename ResultType>
    inline void evalTo(ResultType & result) const
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.rows(), rows());
      PINOCCHIO_CHECK_ARGUMENT_SIZE(result.cols(), cols());

      assert(cols() >= 1);
      assert(rows() >= 1);

      const Eigen::DenseIndex size = m_lhs.rows();
      const auto & b = m_lhs.m_diagonal;
      const auto & c = m_lhs.tridiagonal_matrix.subDiagonal();
      const auto & w = m_lhs.m_sub_diagonal;

      // Forward sweep
      result = m_rhs;
      for (Eigen::DenseIndex i = 1; i < size; ++i)
      {
        result.row(i) -= w[i - 1] * result.row(i - 1);
      }

      // Backward sweep
      result.row(size - 1) /= b[size - 1];
      for (Eigen::DenseIndex i = size - 2; i >= 0; --i)
      {
        result.row(i) -= c[i] * result.row(i + 1);
        result.row(i) /= b[i];
      }
    }

    EIGEN_CONSTEXPR Eigen::Index rows() const EIGEN_NOEXCEPT
    {
      return m_lhs.rows();
    }
    EIGEN_CONSTEXPR Eigen::Index cols() const EIGEN_NOEXCEPT
    {
      return m_rhs.cols();
    }

  protected:
    const TridiagonalSymmetricMatrixInverse & m_lhs;
    const RhsMatrixType & m_rhs;
  };
} // namespace pinocchio

#endif // #ifndef __pinocchio_math_tridiagonal_matrix_hpp__
