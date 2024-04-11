//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_math_lanczos_decomposition_hpp__
#define __pinocchio_math_lanczos_decomposition_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/tridiagonal-matrix.hpp"

namespace pinocchio
{
  
    /// \brief Compute the largest eigenvalues and the associated principle eigenvector via power iteration
  template<typename _Matrix>
  struct LanczosDecompositionTpl
  {
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(_Matrix) PlainMatrix;
    typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(typename PlainMatrix::ColXpr) Vector;
    typedef typename _Matrix::Scalar Scalar;
    enum { Options = _Matrix::Options };
    typedef TridiagonalSymmetricMatrixTpl<Scalar,Options> TridiagonalMatrix;
    
    /// \brief Default constructor for the Lanczos decomposition
    template<typename MatrixLikeType>
    LanczosDecompositionTpl(const MatrixLikeType & mat,
                            const Eigen::DenseIndex decomposition_size)
    : m_Qs(mat.rows(),decomposition_size)
    , m_Ts(decomposition_size)
    , m_A_times_q(mat.rows())
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(mat.rows() == mat.cols(), "The input matrix is not square.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(decomposition_size >= 1, "The size of the decomposition should be greater than one.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(decomposition_size <= mat.rows(), "The size of the decomposition should not be larger than the number of rows.");
      
      compute(mat);
    }
         
    template<typename MatrixLikeType>
    void compute(const MatrixLikeType & A)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(A.rows() == A.cols(), "The input matrix is not square.");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(A.rows() == m_Qs.rows(), "The input matrix is of correct size.");

      const Eigen::DenseIndex decomposition_size = m_Ts.cols();
      auto & alphas = m_Ts.diagonal();
      auto & betas = m_Ts.subDiagonal();
      
      m_Qs.col(0).fill(Scalar(1)/math::sqrt(Scalar(m_Qs.rows())));
      for(Eigen::DenseIndex k = 0; k < decomposition_size; ++k)
      {
        const auto q = m_Qs.col(k);
        m_A_times_q.noalias() = A * q;
        alphas[k] = q.dot(m_A_times_q);
        
        if(k < decomposition_size-1)
        {
          auto q_next = m_Qs.col(k+1);
          m_A_times_q -= alphas[k] * q;
          if(k > 0)
          {
            m_A_times_q -= betas[k-1] * m_Qs.col(k-1);
          }
          betas[k] = m_A_times_q.norm();
//          assert(betas[k] > 1e-14 && "The norm of A*q is too low");
          if(betas[k] <= Eigen::NumTraits<Scalar>::epsilon())
          {
            // We need to sample a new vector, orthogonal to the existing basis
            q_next.setRandom();
            q_next.normalize();
            auto & basis_coefficients = m_A_times_q; // reuse m_A_times_q as temporary variable
            
            basis_coefficients.head(k) = m_Qs.leftCols(k).transpose() * q_next;
            for(Eigen::DenseIndex i = 0; i < k; ++i)
              q_next -= basis_coefficients[i] * q_next;
          }
          else
          {
            q_next.noalias() = m_A_times_q / betas[k];
          }
        }
      }
    }
    
    template<typename MatrixLikeType>
    PlainMatrix computeDecompositionResidual(const MatrixLikeType & A) const
    {
      const Eigen::DenseIndex last_col_id = m_Ts.cols()-1;
      const auto & alphas = m_Ts.diagonal();
      const auto & betas = m_Ts.subDiagonal();
      
      PlainMatrix residual = A * m_Qs;
      residual -= (m_Qs * m_Ts).eval();
      
      const auto & q = m_Qs.col(last_col_id);
      
      m_A_times_q.noalias() = A * q;
      m_A_times_q -= alphas[last_col_id] * q;
      if(last_col_id > 0)
        m_A_times_q -= betas[last_col_id-1] * m_Qs.col(last_col_id-1);
      
      residual.col(last_col_id) -= m_A_times_q;
      
      return residual;
    }
    
    /// \brief Returns the tridiagonal matrix associated with the Lanczos decomposition
    const TridiagonalMatrix & Ts() const { return m_Ts; }
    /// \brief Returns the tridiagonal matrix associated with the Lanczos decomposition
    TridiagonalMatrix & Ts() { return m_Ts; }
    
    /// \brief Returns the orthogonal basis associated with the Lanczos decomposition
    const PlainMatrix & Qs() const { return m_Qs; }
    /// \brief Returns the orthogonal basis associated with the Lanczos decomposition
    PlainMatrix & Qs() { return m_Qs; }

  protected:
    
    PlainMatrix m_Qs;
    TridiagonalMatrix m_Ts;
    mutable Vector m_A_times_q;
  };
}

#endif // ifndef __pinocchio_math_lanczos_decomposition_hpp__
