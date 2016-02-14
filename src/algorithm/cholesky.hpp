//
// Copyright (c) 2015-2016 CNRS
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

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
  
namespace se3
{
  namespace cholesky
  {

    template<typename Mat>
    Mat & solve(const Model & model, const Data & data,
                Eigen::MatrixBase<Mat> & v);

    template<typename Mat>
    Mat & Mv( const Model & model, const Data & data ,
             Eigen::MatrixBase<Mat> & v,
             const bool usingCholesky = false);
    
    template<typename Mat>
    Mat & Uv(const Model & model,
             const Data & data,
             Eigen::MatrixBase<Mat> & v);
    
    template<typename Mat>
    Mat & Utv(const Model & model,
              const Data & data,
              Eigen::MatrixBase<Mat> & v);
    
    template<typename Mat>
    Mat & Uiv(const Model & model,
              const Data & data ,
              Eigen::MatrixBase<Mat> & v);
    
    template<typename Mat>
    Mat & Utiv(const Model & model,
               const Data & data ,
               Eigen::MatrixBase<Mat> & v);
    
    template<typename Mat>
    Mat & solve(const Model & model,
                const Data & data ,
                Eigen::MatrixBase<Mat> & v);

  } // namespace cholesky  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/cholesky.hxx"

#endif // ifndef __se3_cholesky_hpp__
