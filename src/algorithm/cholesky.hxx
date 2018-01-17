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

#ifndef __se3_cholesky_hxx__
#define __se3_cholesky_hxx__

#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace se3 
{
  namespace cholesky
  {

    inline const Eigen::MatrixXd &
    decompose(const Model & model,
              Data & data)
    {
      /*
       *    D = zeros(n,1);
       *    U = eye(n);
       *    for j=n:-1:1
       *      subtree = j+1:tree(j);
       *      D(j) = M(j,j) - U(j,subtree)*diag(D(subtree))*U(j,subtree)';
       *      i=parent(j);
       *      while i>0
       *          U(i,j) = (M(i,j) - U(i,subtree)*diag(D(subtree))*U(j,subtree)') / D(j);
       *          i=parent(i);
       *      end
       *    end
       */
      
#ifndef NDEBUG
      assert(model.check(data) && "data is not consistent with model.");
#endif
      
      const Eigen::MatrixXd & M = data.M;
      Eigen::MatrixXd & U = data.U;
      Eigen::VectorXd & D = data.D;
      Eigen::VectorXd & Dinv = data.Dinv;
      
      for(int j=model.nv-1;j>=0;--j )
      {
        const int NVT = data.nvSubtree_fromRow[(Model::Index)j]-1;
        Eigen::VectorXd::SegmentReturnType DUt = data.tmp.head(NVT);
        if(NVT)
          DUt.noalias() = U.row(j).segment(j+1,NVT).transpose()
          .cwiseProduct(D.segment(j+1,NVT));
        
        D[j] = M(j,j) - U.row(j).segment(j+1,NVT).dot(DUt);
        Dinv[j] = 1./D[j];
        for(int _i = data.parents_fromRow[(Model::Index)j];_i >= 0;_i = data.parents_fromRow[(Model::Index)_i])
          U(_i,j) = (M(_i,j) - U.row(_i).segment(j+1,NVT).dot(DUt)) * Dinv[j];
      }
      
      return data.U;
    }

    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Uv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Uv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Uv<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(v.size() == model.nv);
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
          
          const Eigen::MatrixXd & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=0;k < model.nv-1;++k) // You can stop one step before nv
            v_.row(k) += U.row(k).segment(k+1,nvt[(Model::Index)k]-1) * v_.middleRows(k+1,nvt[(Model::Index)k]-1);
        }
      };
      
    } // namespace internal
    
    /* Compute U*v.
     * Nota: there is no smart way of doing U*D*v, so it is not proposed. */
    template<typename Mat>
    Mat & Uv(const Model & model,
             const Data & data,
             const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      internal::Uv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Utv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Utv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Utv<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(v.size() == model.nv);
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
          
          const Eigen::MatrixXd & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
            v_.segment(k+1,nvt[(Model::Index)k]-1) += U.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v_[k];
            //        v.middleRows(k+1,nvt[(Model::Index)k]-1) += U.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v.row(k);
        }
      };
      
    } // namespace internal

    /* Compute U'*v */
    template<typename Mat>
    Mat & Utv(const Model & model,
              const Data & data,
              const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      internal::Utv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Uiv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Uiv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Uiv<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(v.size() == model.nv);
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
          
          const Eigen::MatrixXd & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
            v_[k] -= U.row(k).segment(k+1,nvt[(Model::Index)k]-1).dot(v_.segment(k+1,nvt[(Model::Index)k]-1));
        }
      };
      
    } // namespace internal
  
    /* Compute U^{-1}*v 
     * Nota: there is no efficient way to compute D^{-1}U^{-1}v
     * in a single loop, so algorithm is not proposed.*/
    template<typename Mat>
    Mat & Uiv(const Model & model,
              const Data & data ,
              const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      internal::Uiv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Utiv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Utiv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Utiv<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(v.size() == model.nv);
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
          
          const Eigen::MatrixXd & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for( int k=0;k<model.nv-1;++k ) // You can stop one step before nv.
            v_.segment(k+1,nvt[(Model::Index)k]-1) -= U.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose() * v_[k];
          //        v.middleRows(k+1,nvt[(Model::Index)k]-1).transpose() -= v.row(k).transpose()*();
        }
      };
      
    } // namespace internal

    template<typename Mat>
    Mat & Utiv(const Model & model,
               const Data & data ,
               const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      internal::Utiv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }

    namespace internal
    {
      template<typename Mat, typename MatRes, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Mv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & min,
                        const Eigen::MatrixBase<MatRes> & mout
                        )
        {
          MatRes & mout_ = const_cast<Eigen::MatrixBase<MatRes> &>(mout).derived();
          for(int k = 0; k < min.cols(); ++k)
            cholesky::Mv(model,data,min.col(k),mout_.col(k));
        }
      };
      
      template<typename Mat, typename MatRes>
      struct Mv<Mat,MatRes,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & vin,
                        const Eigen::MatrixBase<MatRes> & vout)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRes)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(vin.size() == model.nv);
          assert(vout.size() == model.nv);
          MatRes & vout_ = const_cast<Eigen::MatrixBase<MatRes> &>(vout).derived();
          
          const Eigen::MatrixXd & M = data.M;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=model.nv-1;k>=0;--k)
          {
            vout_[k] = M.row(k).segment(k,nvt[(Model::Index)k]) * vin.segment(k,nvt[(Model::Index)k]);
            vout_.segment(k+1,nvt[(Model::Index)k]-1) += M.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*vin[k];
//            res.row(k) = M.row(k).segment(k,nvt[(Model::Index)k]) * v.middleRows(k,nvt[(Model::Index)k]);
//            res.middleRows(k+1,nvt[(Model::Index)k]-1) += M.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v.row(k);
          }
        }
      };
      
    } // namespace internal
    
    template<typename Mat, typename MatRes>
    MatRes & Mv(const Model & model,
                const Data & data,
                const Eigen::MatrixBase<Mat> & min,
                const Eigen::MatrixBase<MatRes> & mout)
    {
      MatRes & mout_ = const_cast<Eigen::MatrixBase<MatRes> &>(mout).derived();
      internal::Mv<Mat,MatRes,Mat::ColsAtCompileTime>::run(model,data,min.derived(),mout_);
      return mout_.derived();
    }
    
    template<typename Mat>
    typename EIGEN_PLAIN_TYPE(Mat) Mv(const Model & model,
                                      const Data & data,
                                      const Eigen::MatrixBase<Mat> & min)
    {
      typedef typename EIGEN_PLAIN_TYPE(Mat) ReturnType;
      ReturnType res(model.nv,min.cols());
      return Mv(model,data,min,res);
    }
    
//    template<typename Mat>
//    Mat & Mv(const Model & model,
//             const Data & data,
//             const Eigen::MatrixBase<Mat> & v,
//             const bool usingCholesky)
//    {
//#ifndef NDEBUG
//      assert(model.check(data) && "data is not consistent with model.");
//#endif
//      Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
//      if(usingCholesky) internal::UDUtv(model,data,v_);
//      else v_ = internal::Mv(model,data,v_);
//
//      return v_.derived();
//    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct UDUtv
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::UDUtv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct UDUtv<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          assert(v.size() == model.nv);
          
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();

          cholesky::Utv(model,data,v_);
          v_.array() *= data.D.array();
          cholesky::Uv(model,data,v_);
        }
      };
      
    } // namespace internal
    
    template<typename Mat>
    Mat & UDUtv(const Model & model,
                const Data & data,
                const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      
      internal::UDUtv<Mat>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct solve
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::solve(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct solve<Mat,1>
      {
        static void run(const Model & model,
                        const Data & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
#ifndef NDEBUG
          assert(model.check(data) && "data is not consistent with model.");
#endif
          
          Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
          
          cholesky::Uiv(model,data,v_);
          v_.array() *= data.Dinv.array();
          cholesky::Utiv(model,data,v_);
        }
      };
      
    } // namespace internal
    
    template<typename Mat>
    Mat & solve(const Model & model,
                const Data & data ,
                const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = const_cast<Eigen::MatrixBase<Mat> &>(m).derived();
      internal::solve<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat>
      Mat & Miunit(const Model & model,
                   const Data & data,
                   const int col,
                   const Eigen::MatrixBase<Mat> & v)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        
#ifndef NDEBUG
        assert(model.check(data) && "data is not consistent with model.");
#endif
        assert(col < model.nv);
        assert(v.rows() == model.nv);
        
        const Eigen::MatrixXd & U = data.U;
        const std::vector<int> & nvt = data.nvSubtree_fromRow;
        Mat & v_ = const_cast<Eigen::MatrixBase<Mat> &>(v).derived();
        
        const int last_col = std::min(col-1,model.nv-2); // You can start from nv-2 (no child in nv-1)
        v_[col] = 1.;
        for( int k=last_col;k>=0;--k )
        {
          int nvt_max = std::min(col,nvt[(Model::Index)k]-1);
          v_[k] -= U.row(k).segment(k+1,nvt_max).dot(v_.segment(k+1,nvt_max));
        }
        
        v_.head(col+1).array() *= data.Dinv.head(col+1).array();
        
        for( int k=0;k<model.nv-1;++k ) // You can stop one step before nv.
        {
          int nvt_max = nvt[(Model::Index)k]-1;
          v_.segment(k+1,nvt_max) -= U.row(k).segment(k+1,nvt_max).transpose() * v_[k];
        }
        
        return v_.derived();
      }
    }// namespace internal
    
    template<typename Mat>
    Mat & computeMinv(const Model & model,
                      const Data & data,
                      const Eigen::MatrixBase<Mat> & Minv)
    {
      assert(Minv.rows() == model.nv);
      assert(Minv.cols() == model.nv);
      
#ifndef NDEBUG
      assert(model.check(data) && "data is not consistent with model.");
#endif
      
      Mat & Minv_ = const_cast<Eigen::MatrixBase<Mat> &>(Minv).derived();
      
      for(int k = 0; k < model.nv; ++k)
        internal::Miunit(model,data,k,Minv_.col(k));
      
      return Minv_;
    }

  } //   namespace cholesky
} // namespace se3

/// @endcond

#endif // ifndef __se3_cholesky_hxx__
