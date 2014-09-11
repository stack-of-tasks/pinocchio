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
    inline const Eigen::MatrixXd&
    decompose(const Model & model, 
	      Data&         data );

    template<typename Mat>
    Mat & solve( const Model &            model, 
		 const Data&              data ,
		 Eigen::MatrixBase<Mat> & v);

    template<typename Mat>
    Mat & Mv( const Model &              model, 
	      const  Data&               data ,
	      Eigen::MatrixBase<Mat> &   v,
	      bool usingCholesky = false);

  } // namespace cholesky  
} // namespace se3 

/* --- Details -------------------------------------------------------------------- */

namespace se3 
{
  namespace cholesky
  {

    inline const Eigen::MatrixXd&
    decompose(const Model &           model, 
	      Data&                   data )
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
	  const int NVT = data.nvSubtree_fromRow[j]-1;
	  Eigen::VectorXd::SegmentReturnType DUt = data.tmp.head(NVT);
	  if(NVT)
	    DUt = U.row(j).segment(j+1,NVT).transpose()
	      .cwiseProduct(D.segment(j+1,NVT));
	
	  D[j] = M(j,j) - U.row(j).segment(j+1,NVT) * DUt;
	
	  for( int _i=data.parents_fromRow[j];_i>=0;_i=data.parents_fromRow[_i] )
	    U(_i,j) = (M(_i,j) - U.row(_i).segment(j+1,NVT).dot(DUt)) / D[j]; 
	}

      return data.U;
    }

    /* Compute U*v.
     * Nota: there is no smart way of doing U*D*v, so it is not proposed. */
    template<typename Mat>
    Mat & 
    Uv( const Model &                  model, 
	const Data&                    data ,
	Eigen::MatrixBase<Mat> &   v)
    {
      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;

      for( int k=0;k<model.nv-1;++k ) // You can stop one step before nv
	v[k] += U.row(k).segment(k+1,nvt[k]-1) * v.segment(k+1,nvt[k]-1);

      return v.derived();
    }

    /* Compute U'*v */
    template<typename Mat>
    Mat &
    Utv( const Model &                  model, 
	 const Data&                          data ,
	 Eigen::MatrixBase<Mat> &   v)
    {
      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      for( int i=model.nv-2;i>=0;--i ) // You can start from nv-2 (no child in nv-1)
	v.segment(i+1,nvt[i]-1) += U.row(i).segment(i+1,nvt[i]-1).transpose()*v[i];
      
      return v.derived();
    }
  
    /* Compute U^{-1}*v 
     * Nota: there is no efficient way to compute D^{-1}U^{-1}v
     * in a single loop, so algorithm is not proposed.*/
    template<typename Mat>
    Mat &
    Uiv( const Model &                  model, 
	 const Data&                          data ,
	 Eigen::MatrixBase<Mat> &   v)
    {
      /* We search y s.t. v = U y. 
       * For any k, v_k = y_k + U_{k,k+1:} y_{k+1:} */
      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;

      for( int k=model.nv-2;k>=0;--k ) // You can start from nv-2 (no child in nv-1)
	v[k] -= U.row(k).segment(k+1,nvt[k]-1) * v.segment(k+1,nvt[k]-1);
      return v.derived();
    }

    template<typename Mat>
    Mat &
    Utiv( const Model &                  model, 
	  const  Data&                          data ,
	  Eigen::MatrixBase<Mat> &   v)
    {
      /* We search y s.t. v = U' y. 
       * For any k, v_k = y_k + sum_{m \in parent{k}} U(m,k) v(k). */

      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
      
      const Eigen::MatrixXd & U = data.U;
      const std::vector<int> & nvt = data.nvSubtree_fromRow;
      for( int i=0;i<model.nv-1;++i ) // You can stop one step before nv.
	v.segment(i+1,nvt[i]-1) -= U.row(i).segment(i+1,nvt[i]-1).transpose()*v[i];

      return v.derived();
    }

    namespace internal
    {
      template<typename Mat>
      Mat
      Mv( const Model &                  model, 
	  const  Data&                          data ,
	  const Eigen::MatrixBase<Mat> &   v)
      {
	assert(v.size() == model.nv);
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
      
	const Eigen::MatrixXd & M = data.M;
	const std::vector<int> & nvt = data.nvSubtree_fromRow;
	Mat res(model.nv);

	for( int k=model.nv-1;k>=0;--k ) 
	  {
	    res[k] = M.row(k).segment(k,nvt[k]) * v.segment(k,nvt[k]);
	    res.segment(k+1,nvt[k]-1) += M.row(k).segment(k+1,nvt[k]-1).transpose()*v[k];
	  }

	return res;
      }

      template<typename Mat>
      Mat &
      UDUtv( const Model &              model, 
	     const Data&                data ,
	     Eigen::MatrixBase<Mat> &   v)
      {
	Utv(model,data,v);
	for( int k=0;k<model.nv;++k ) v[k] *= data.D[k];
	return Uv(model,data,v);
      }
    } // internal
  
    template<typename Mat>
    Mat &
    Mv( const Model &                  model, 
	const  Data&                          data ,
	Eigen::MatrixBase<Mat> &   v,
	bool usingCholesky = false)
    {
      if(usingCholesky) return internal::UDUtv(model,data,v);
      else return v = internal::Mv(model,data,v);
    }

    template<typename Mat>
    Mat &
    solve( const Model &              model, 
	   const Data&                data ,
	   Eigen::MatrixBase<Mat> &   v)
    {
      Uiv(model,data,v);
      for( int k=0;k<model.nv;++k ) v[k] /= data.D[k];
      return Utiv(model,data,v);
    }

  } //   namespace cholesky
} // namespace se3 
#endif // ifndef __se3_cholesky_hpp__
