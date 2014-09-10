#ifndef __se3_cholesky_hpp__
#define __se3_cholesky_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
  
namespace se3
{
  inline const Eigen::MatrixXd&
  cholesky(const Model &           model, 
	   Data&                   data );


  template<typename D>
  void multSqrt( const Model &                  model, 
		 Data&                          data ,
		 const Eigen::MatrixBase<D> &   v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */

namespace se3 
{

  inline const Eigen::MatrixXd&
  cholesky(const Model &           model, 
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

    U = Eigen::MatrixXd::Identity(model.nv,model.nv);

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

  template<typename D>
  void multSqrt( const Model &                  model, 
		 Data&                          data ,
		 const Eigen::MatrixBase<D> &   v)
  {
    /* for i = n:1
     *    res[i] = L(i,i) x[i]
     *    while j>0
     *       y[i] += L(i,j) x[j]
     */

    


  }
  


} // namespace se3 
#endif // ifndef __se3_cholesky_hpp__
