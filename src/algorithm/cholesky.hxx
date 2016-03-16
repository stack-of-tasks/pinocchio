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

#ifndef __se3_cholesky_hxx__
#define __se3_cholesky_hxx__

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
      
      Eigen::MatrixXd & M = data.M;
      Eigen::MatrixXd & U = data.U;
      Eigen::VectorXd & D = data.D;
      
      for(int j=model.nv-1;j>=0;--j )
      {
        const int NVT = data.nvSubtree_fromRow[(Model::Index)j]-1;
        Eigen::VectorXd::SegmentReturnType DUt = data.tmp.head(NVT);
        if(NVT)
          DUt = U.row(j).segment(j+1,NVT).transpose()
          .cwiseProduct(D.segment(j+1,NVT));
        
        D[j] = M(j,j) - U.row(j).segment(j+1,NVT) * DUt;
        
        for(int _i = data.parents_fromRow[(Model::Index)j];_i >= 0;_i = data.parents_fromRow[(Model::Index)_i])
          U(_i,j) = (M(_i,j) - U.row(_i).segment(j+1,NVT).dot(DUt)) / D[j];
      }
      
      return data.U;
    }

    /* Compute U*v.
     * Nota: there is no smart way of doing U*D*v, so it is not proposed. */
    template<typename Mat>
    Mat & Uv(const Model & model,
             const Data & data,
             Eigen::MatrixBase<Mat> & v)
    {
      assert(v.rows() == model.nv);
      
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      
      for(int k=0;k < model.nv-1;++k) // You can stop one step before nv
        v.row(k) += U.row(k).segment(k+1,nvt[(Model::Index)k]-1) * v.middleRows(k+1,nvt[(Model::Index)k]-1);
      
      return v.derived();
    }

    /* Compute U'*v */
    template<typename Mat>
    Mat & Utv(const Model & model,
              const Data & data,
              Eigen::MatrixBase<Mat> & v)
    {
      assert(v.rows() == model.nv);
      
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
        v.middleRows(k+1,nvt[(Model::Index)k]-1) += U.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v.row(k);
      
      return v.derived();
    }
  
    /* Compute U^{-1}*v 
     * Nota: there is no efficient way to compute D^{-1}U^{-1}v
     * in a single loop, so algorithm is not proposed.*/
    template<typename Mat>
    Mat & Uiv(const Model & model,
              const Data & data ,
              Eigen::MatrixBase<Mat> & v)
    {
      /* We search y s.t. v = U y. 
       * For any k, v_k = y_k + U_{k,k+1:} y_{k+1:} */
      assert(v.rows() == model.nv);
      
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      
      for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
        v.row(k) -= U.row(k).segment(k+1,nvt[(Model::Index)k]-1) * v.middleRows(k+1,nvt[(Model::Index)k]-1);
      return v.derived();
    }

    template<typename Mat>
    Mat & Utiv(const Model & model,
               const Data & data ,
               Eigen::MatrixBase<Mat> & v)
    {
      /* We search y s.t. v = U' y. 
       * For any k, v_k = y_k + sum_{m \in parent{k}} U(m,k) v(k). */
      assert(v.rows() == model.nv);
      
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      for( int k=0;k<model.nv-1;++k ) // You can stop one step before nv.
        v.middleRows(k+1,nvt[(Model::Index)k]-1) -= U.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v.row(k);

      return v.derived();
    }

    namespace internal
    {
      template<typename Mat>
      Mat Mv(const Model & model,
             const Data & data,
             const Eigen::MatrixBase<Mat> & v)
      {
        assert(v.rows() == model.nv);
        
        const Eigen::MatrixXd & M = data.M;
        const std::vector<int> & nvt = data.nvSubtree_fromRow;
        Mat res(model.nv);
        
        for(int k=model.nv-1;k>=0;--k)
        {
          res.row(k) = M.row(k).segment(k,nvt[(Model::Index)k]) * v.middleRows(k,nvt[(Model::Index)k]);
          res.middleRows(k+1,nvt[(Model::Index)k]-1) += M.row(k).segment(k+1,nvt[(Model::Index)k]-1).transpose()*v.row(k);
        }
        
        return res;
      }
      
      template<typename Mat>
      Mat & UDUtv(const Model & model,
                  const Data & data,
                  Eigen::MatrixBase<Mat> & v)
      {
        Utv(model,data,v);
        for( int k=0;k<model.nv;++k ) v.row(k) *= data.D[k];
        return Uv(model,data,v);
      }
    } // internal
    
    template<typename Mat>
    Mat & Mv(const Model & model,
             const Data & data,
             Eigen::MatrixBase<Mat> & v,
             const bool usingCholesky)
    {
      if(usingCholesky) return internal::UDUtv(model,data,v);
      else return v = internal::Mv(model,data,v);
    }
    
    template<typename Mat>
    Mat & solve(const Model & model,
                const Data & data ,
                Eigen::MatrixBase<Mat> & v)
    {
      Uiv(model,data,v);
      for(int k=0;k<model.nv;++k) v.row(k) /= data.D[k];
      return Utiv(model,data,v);
    }

  } //   namespace cholesky
} // namespace se3

/// @endcond

#endif // ifndef __se3_cholesky_hxx__
