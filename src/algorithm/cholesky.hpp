//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_cholesky_hpp__
#define __se3_cholesky_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
  
namespace se3
{
  namespace cholesky
  {
    
    ///
    /// \brief Compute the Cholesky decomposition of the joint space inertia matrix M contained in data.
    ///
    /// \note The Cholesky decomposition corresponds to
    ///       \f$ M = U D U^{\top}\f$ with \f$U\f$ an upper triangular matrix with ones on its main diagonal and \f$D\f$ a diagonal matrix.
    ///
    ///       The result stored in data.U and data.D matrices. One can retrieve the matrice M by performing the
    ///       computation data.U * data.D * data.U.transpose()
    ///
    ///       See https://en.wikipedia.org/wiki/Cholesky_decomposition for futher details.
    ///
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    ///
    /// \return A reference to the upper triangular matrix \f$U\f$.
    ///
    inline const Eigen::MatrixXd &
    decompose(const Model & model,
              Data & data);

    ///
    /// \brief Return the solution \f$x\f$ of \f$ M x = y \f$ using the Cholesky decomposition stored in data given the entry \f$ y \f$. Act like solveInPlace of Eigen::LLT.
    ///
    /// \note This algorithm is useful to compute the forward dynamics, retriving the joint acceleration \f$ \ddot{q} \f$ from the current joint torque \f$ \tau \f$
    ///       \f$
    ///           M(q) \ddot{q} + b(q, \dot{q}) = \tau \iff \ddot{q} = M(q)^{-1} (\tau - b(q, \dot{q}))
    ///       \f$
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] y The input matrix to inverse which also contains the result \f$x\f$ of the inversion.
    ///
    template<typename Mat>
    Mat & solve(const Model & model, const Data & data,
                const Eigen::MatrixBase<Mat> & y);

    ///
    /// \brief Performs the multiplication \f$ M v \f$ by using the sparsity pattern of the M matrix.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[in] min The input matrix to multiply with data.M.
    ///
    /// \return A the result of \f$ Mv \f$.
    ///
    template<typename Mat>
    typename EIGEN_PLAIN_TYPE(Mat) Mv(const Model & model,
                                      const Data & data,
                                      const Eigen::MatrixBase<Mat> & min);
    
    ///
    /// \brief Performs the multiplication \f$ M v \f$ by using the sparsity pattern of the M matrix.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[in] min The input matrix to multiply with data.M.
    /// \param[out] mout The output matrix where the result of \f$ Mv \f$ is stored.
    ///
    /// \return A reference of the result of \f$ Mv \f$.
    ///
    template<typename Mat, typename MatRes>
    MatRes & Mv(const Model & model,
                const Data & data,
                const Eigen::MatrixBase<Mat> & min,
                const Eigen::MatrixBase<MatRes> & mout);
    
    
    ///
    /// \brief Performs the multiplication \f$ M v \f$ by using the Cholesky decomposition of M stored in data.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] m The input matrix where the result of \f$ Mv \f$ is stored.
    ///
    /// \return A reference of the result of \f$ Mv \f$.
    ///
    template<typename Mat>
    Mat & UDUtv(const Model & model,
                const Data & data,
                const Eigen::MatrixBase<Mat> & m);
    
    ///
    /// \brief Perform the sparse multiplication \f$ Uv \f$ using the Cholesky decomposition stored in data and acting in place.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] v The input matrix to multiply with data.U and also storing the result.
    ///
    /// \return A reference to the result of \f$ Uv \f$ stored in v.
    ///
    template<typename Mat>
    Mat & Uv(const Model & model,
             const Data & data,
             const Eigen::MatrixBase<Mat> & v);
    
    ///
    /// \brief Perform the sparse multiplication \f$ U^{\top}v \f$ using the Cholesky decomposition stored in data and acting in place.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] v The input matrix to multiply with data.U.tranpose() and also storing the result.
    ///
    /// \return A reference to the result of \f$ U^{\top}v \f$ stored in v.
    ///
    template<typename Mat>
    Mat & Utv(const Model & model,
              const Data & data,
              const Eigen::MatrixBase<Mat> & v);
    
    ///
    /// \brief Perform the pivot inversion \f$ U^{-1}v \f$ using the Cholesky decomposition stored in data and acting in place.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] v The input matrix to multiply with data.U^{-1} and also storing the result.
    ///
    /// \return A reference to the result of \f$ U^{-1}v \f$ stored in v.
    ///
    /// \remark The result is similar to the code data.U.triangularView<Eigen::Upper> ().solveInPlace(v).
    ///
    template<typename Mat>
    Mat & Uiv(const Model & model,
              const Data & data ,
              const Eigen::MatrixBase<Mat> & v);
    
    ///
    /// \brief Perform the pivot inversion \f$ U^{-\top}v \f$ using the Cholesky decomposition stored in data and acting in place.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] v The input matrix to multiply with data.U^{-\top} and also storing the result.
    ///
    /// \return A reference to the result of \f$ U^{-\top}v \f$ stored in v.
    ///
    /// \remark The result is similar to the code data.U.triangularView<Eigen::Upper> ().transpose().solveInPlace(v).
    ///
    template<typename Mat>
    Mat & Utiv(const Model & model,
               const Data & data ,
               const Eigen::MatrixBase<Mat> & v);
    
    ///
    /// \brief Perform the sparse inversion \f$ M^{-1}v \f$ using the Cholesky decomposition stored in data and acting in place.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[inout] v The input matrix to multiply with data.M^{-1} and also storing the result.
    ///
    /// \return A reference to the result of \f$ M^{-1}v \f$ stored in v.
    ///
    template<typename Mat>
    Mat & solve(const Model & model,
                const Data & data ,
                const Eigen::MatrixBase<Mat> & v);
    
    ///
    /// \brief PComputes the inverse of the joint inertia matrix M from its Cholesky factorization.
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] data The data structure of the rigid body system.
    /// \param[out] Minv The output matrix where the result is stored.
    ///
    /// \return A reference to the result.
    ///
    template<typename Mat>
    Mat & computeMinv(const Model & model,
                      const Data & data,
                      const Eigen::MatrixBase<Mat> & Minv);
    
  } // namespace cholesky  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/cholesky.hxx"

#endif // ifndef __se3_cholesky_hpp__
