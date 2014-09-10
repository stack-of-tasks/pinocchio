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
    decompose(const Model &           model, 
	      Data&                   data );
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

  /* Compute U*D*v */
  template<typename Mat>
  Eigen::VectorXd
  UDv( const Model &                  model, 
       Data&                          data ,
       const Eigen::MatrixBase<Mat> &   v)
  {
    /* for i = n:1
     *    res[i] = L(i,i) x[i]
     *    while j>0
     *       y[i] += L(i,j) x[j]
     */
    assert(v.size() == model.nv);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    Eigen::VectorXd res = D.asDiagonal()*v;

    for( int j=0;j<model.nv;++j )
      res[j] += U.row(j).segment(j+1,data.nvSubtree_fromRow[j]-1)
	* res.segment(j+1,data.nvSubtree_fromRow[j]-1);

    return res;
  }
  
  /* Compute U*v */
  template<typename Mat>
  Eigen::VectorXd
  Uv( const Model &                  model, 
      Data&                          data ,
      const Eigen::MatrixBase<Mat> &   v)
  {
    assert(v.size() == model.nv);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    Eigen::VectorXd res = v;

    for( int j=0;j<model.nv;++j )
      res[j] += U.row(j).segment(j+1,data.nvSubtree_fromRow[j]-1)
	* res.segment(j+1,data.nvSubtree_fromRow[j]-1);

    return res;
  }
  
  /* Compute D*U'*v */
  template<typename Mat>
  Eigen::VectorXd
  DUtv( const Model &                  model, 
	    Data&                          data ,
	    const Eigen::MatrixBase<Mat> &   v)
  {
    assert(v.size() == model.nv);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    Eigen::VectorXd res = v;
    const std::vector<int> & parents = data.parents_fromRow;

    for( int i=model.nv-1;i>=0;--i )
      {
	for( Model::Index j = parents[i];j>=0;j=parents[j] )
	  res[i] += U(j,i)*v[j];
	res[i] *= D[i];
      }

    return res;
  }

  /* Compute U'*v */
  template<typename Mat>
  Eigen::VectorXd
  Utv( const Model &                  model, 
       Data&                          data ,
       const Eigen::MatrixBase<Mat> &   v)
  {
    assert(v.size() == model.nv);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    Eigen::VectorXd res = v;
    const std::vector<int> & parents = data.parents_fromRow;

    for( int i=model.nv-1;i>=0;--i )
      for( Model::Index j = parents[i];j>=0;j=parents[j] )
	res[i] += U(j,i)*v[j];

    return res;
  }
  
  /* Compute U^{-1}*v 
   * Nota: there is no efficient way to compute D^{-1}U^{-1}v
   * in a single loop, so algorithm is not proposed.*/
  template<typename Mat>
  Eigen::VectorXd
  Uiv( const Model &                  model, 
       Data&                          data ,
       const Eigen::MatrixBase<Mat> &   v)
  {
    /* We search y s.t. v = U y. 
     * For any k, v_k = y_k + U_{k,k+1:} y_{k+1:} */
    assert(v.size() == model.nv);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);

    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd res = v;
    const std::vector<int> & nvt = data.nvSubtree_fromRow;

    for( int k=model.nv-1;k>=0;--k )
      res[k] -= U.row(k).segment(k+1,nvt[k]-1).dot(res.segment(k+1,nvt[k]-1));
    return res;
  }

    template<typename Mat>
    Eigen::VectorXd
    Utiv( const Model &                  model, 
	  Data&                          data ,
	  const Eigen::MatrixBase<Mat> &   v)
    {
      /* We search y s.t. v = U' y. 
       * For any k, v_k = y_k + sum_{m \in parent{k}} U(m,k) v(k). */

      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
      
      Eigen::MatrixXd & U = data.U;
      Eigen::VectorXd res = v;
      const std::vector<int> & parents = data.parents_fromRow;

      for( int k=1;k<model.nv;++k )
	for( int m = parents[k];m>=0;m=parents[m] )
	  res[k] -= U(m,k)*res[m];

      return res;
  }

    template<typename Mat>
    Eigen::VectorXd
    UtiDiv( const Model &                  model, 
	  Data&                          data ,
	  const Eigen::MatrixBase<Mat> &   v)
    {
      /* We search y s.t. v = U' y. 
       * For any k, v_k = y_k + sum_{m \in parent{k}} U(m,k) v(k). */

      assert(v.size() == model.nv);
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
      
      const Eigen::MatrixXd & U = data.U;
      const Eigen::VectorXd & D = data.D;
      Eigen::VectorXd res = v.cwiseQuotient(D);
      const std::vector<int> & parents = data.parents_fromRow;

      for( int k=1;k<model.nv;++k )
	for( int m = parents[k];m>=0;m=parents[m] )
	  res[k] -= U(m,k)*res[m];

      return res;
  }

 

  } //   namespace cholesky
} // namespace se3 
#endif // ifndef __se3_cholesky_hpp__
