//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_cholesky_hxx__
#define __pinocchio_cholesky_hxx__

#include "pinocchio/algorithm/check.hpp"

/// @cond DEV

namespace pinocchio 
{
  namespace cholesky
  {

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::MatrixXs &
    decompose(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              DataTpl<Scalar,Options,JointCollectionTpl> & data)
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
      PINOCCHIO_UNUSED_VARIABLE(model);
      assert(model.check(data) && "data is not consistent with model.");
      
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

      const typename Data::MatrixXs & M = data.M;
      typename Data::MatrixXs & U = data.U;
      typename Data::VectorXs & D = data.D;
      typename Data::VectorXs & Dinv = data.Dinv;
      
      for(int j=model.nv-1;j>=0;--j )
      {
        const int NVT = data.nvSubtree_fromRow[(size_t)j]-1;
        typename Data::VectorXs::SegmentReturnType DUt = data.tmp.head(NVT);
        if(NVT)
          DUt.noalias() = U.row(j).segment(j+1,NVT).transpose()
          .cwiseProduct(D.segment(j+1,NVT));
        
        D[j] = M(j,j) - U.row(j).segment(j+1,NVT).dot(DUt);
        Dinv[j] = Scalar(1)/D[j];
        for(int _i = data.parents_fromRow[(size_t)j]; _i >= 0;_i = data.parents_fromRow[(size_t)_i])
          U(_i,j) = (M(_i,j) - U.row(_i).segment(j+1,NVT).dot(DUt)) * Dinv[j];
      }
      
      return data.U;
    }

    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Uv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Uv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Uv<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
          
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          PINOCCHIO_UNUSED_VARIABLE(model);
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
          
          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
          
          const typename Data::MatrixXs & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=0;k < model.nv-1;++k) // You can stop one step before nv
            v_.row(k) += U.row(k).segment(k+1,nvt[(size_t)k]-1) * v_.middleRows(k+1,nvt[(size_t)k]-1);
        }
      };
      
    } // namespace internal
    
    /* Compute U*v.
     * Nota: there is no smart way of doing U*D*v, so it is not proposed. */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & Uv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             const DataTpl<Scalar,Options,JointCollectionTpl> & data,
             const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      internal::Uv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Utv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Utv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Utv<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
          
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          
          PINOCCHIO_UNUSED_VARIABLE(model);
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
          
          const typename Data::MatrixXs & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
            v_.segment(k+1,nvt[(size_t)k]-1) += U.row(k).segment(k+1,nvt[(size_t)k]-1).transpose()*v_[k];
        }
      };
      
    } // namespace internal

    /* Compute U'*v */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & Utv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
              const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      internal::Utv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Uiv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Uiv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Uiv<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
          
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
          
          const typename Data::MatrixXs & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=model.nv-2;k>=0;--k) // You can start from nv-2 (no child in nv-1)
            v_[k] -= U.row(k).segment(k+1,nvt[(size_t)k]-1).dot(v_.segment(k+1,nvt[(size_t)k]-1));
        }
      };
      
    } // namespace internal
  
    /* Compute U^{-1}*v 
     * Nota: there is no efficient way to compute D^{-1}U^{-1}v
     * in a single loop, so algorithm is not proposed.*/
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & Uiv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
              const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      internal::Uiv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Utiv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::Utiv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct Utiv<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
          
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
          
          const typename Data::MatrixXs & U = data.U;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=0; k<model.nv-1; ++k) // You can stop one step before nv.
            v_.segment(k+1,nvt[(size_t)k]-1) -= U.row(k).segment(k+1,nvt[(size_t)k]-1).transpose() * v_[k];
        }
      };
      
    } // namespace internal

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & Utiv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      internal::Utiv<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }

    namespace internal
    {
      template<typename Mat, typename MatRes, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct Mv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & min,
                        const Eigen::MatrixBase<MatRes> & mout
                        )
        {
          MatRes & mout_ = PINOCCHIO_EIGEN_CONST_CAST(MatRes,mout);
          for(int k = 0; k < min.cols(); ++k)
            cholesky::Mv(model,data,min.col(k),mout_.col(k));
        }
      };
      
      template<typename Mat, typename MatRes>
      struct Mv<Mat,MatRes,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & vin,
                        const Eigen::MatrixBase<MatRes> & vout)
        {
          typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
          
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRes)
          
          PINOCCHIO_UNUSED_VARIABLE(model);
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(vin.size(), model.nv);
          PINOCCHIO_CHECK_ARGUMENT_SIZE(vout.size(), model.nv);
          
          MatRes & vout_ = PINOCCHIO_EIGEN_CONST_CAST(MatRes,vout);
          
          const typename Data::MatrixXs & M = data.M;
          const std::vector<int> & nvt = data.nvSubtree_fromRow;
          
          for(int k=model.nv-1;k>=0;--k)
          {
            vout_[k] = M.row(k).segment(k,nvt[(size_t)k]) * vin.segment(k,nvt[(size_t)k]);
            vout_.segment(k+1,nvt[(size_t)k]-1) += M.row(k).segment(k+1,nvt[(size_t)k]-1).transpose()*vin[k];
          }
        }
      };
      
    } // namespace internal
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat, typename MatRes>
    MatRes & Mv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                const Eigen::MatrixBase<Mat> & min,
                const Eigen::MatrixBase<MatRes> & mout)
    {
      MatRes & mout_ = PINOCCHIO_EIGEN_CONST_CAST(MatRes,mout);
      internal::Mv<Mat,MatRes,Mat::ColsAtCompileTime>::run(model,data,min.derived(),mout_);
      return mout_.derived();
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    typename PINOCCHIO_EIGEN_PLAIN_TYPE(Mat) Mv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                      const Eigen::MatrixBase<Mat> & min)
    {
      typedef typename PINOCCHIO_EIGEN_PLAIN_TYPE(Mat) ReturnType;
      ReturnType res(model.nv,min.cols());
      return Mv(model,data,min,res);
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct UDUtv
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::UDUtv(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct UDUtv<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          
          assert(model.check(data) && "data is not consistent with model.");
          PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
          
          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);

          cholesky::Utv(model,data,v_);
          v_.array() *= data.D.array();
          cholesky::Uv(model,data,v_);
        }
      };
      
    } // namespace internal
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & UDUtv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      
      internal::UDUtv<Mat>::run(model,data,m_);
      return m_;
    }
    
    namespace internal
    {
      template<typename Mat, int ColsAtCompileTime = Mat::ColsAtCompileTime>
      struct solve
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & m)
        {
          Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
          for(int k = 0; k < m_.cols(); ++k)
            cholesky::solve(model,data,m_.col(k));
        }
      };
      
      template<typename Mat>
      struct solve<Mat,1>
      {
        template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
        static void run(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                        const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                        const Eigen::MatrixBase<Mat> & v)
        {
          EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat)
          
          assert(model.check(data) && "data is not consistent with model.");

          Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
          
          cholesky::Uiv(model,data,v_);
          v_.array() *= data.Dinv.array();
          cholesky::Utiv(model,data,v_);
        }
      };
      
    } // namespace internal
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & solve(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                const Eigen::MatrixBase<Mat> & m)
    {
      Mat & m_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,m);
      internal::solve<Mat,Mat::ColsAtCompileTime>::run(model,data,m_);
      return m_.derived();
    }
    
    namespace internal
    {
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
      Mat & Miunit(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                   const int col,
                   const Eigen::MatrixBase<Mat> & v)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        
        PINOCCHIO_UNUSED_VARIABLE(model);
        assert(model.check(data) && "data is not consistent with model.");
        PINOCCHIO_CHECK_INPUT_ARGUMENT(col < model.nv && col >= 0);
        PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), model.nv);
        
        typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
        
        const typename Data::MatrixXs & U = data.U;
        const std::vector<int> & nvt = data.nvSubtree_fromRow;
        Mat & v_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,v);
        
        const int last_col = std::min<int>(col-1,model.nv-2); // You can start from nv-2 (no child in nv-1)
        v_.tail(model.nv - col - 1).setZero();
        v_[col] = 1.;
        for( int k=last_col;k>=0;--k )
        {
          int nvt_max = std::min<int>(col,nvt[(size_t)k]-1);
          v_[k] = -U.row(k).segment(k+1,nvt_max).dot(v_.segment(k+1,nvt_max));
        }
        
        v_.head(col+1).array() *= data.Dinv.head(col+1).array();
        
        for( int k=0;k<model.nv-1;++k ) // You can stop one step before nv.
        {
          int nvt_max = nvt[(size_t)k]-1;
          v_.segment(k+1,nvt_max) -= U.row(k).segment(k+1,nvt_max).transpose() * v_[k];
        }
        
        return v_.derived();
      }
    }// namespace internal
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Mat>
    Mat & computeMinv(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                      const Eigen::MatrixBase<Mat> & Minv)
    {
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Minv.rows(), model.nv);
      PINOCCHIO_CHECK_ARGUMENT_SIZE(Minv.cols(), model.nv);
      
      assert(model.check(data) && "data is not consistent with model.");

      Mat & Minv_ = PINOCCHIO_EIGEN_CONST_CAST(Mat,Minv);
      
      for(int k = 0; k < model.nv; ++k)
        internal::Miunit(model,data,k,Minv_.col(k));
      
      return Minv_.derived();
    }

  } //   namespace cholesky
} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_cholesky_hxx__
