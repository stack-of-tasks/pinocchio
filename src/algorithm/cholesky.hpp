#ifndef __se3_cholesky_hpp__
#define __se3_cholesky_hpp__

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
  struct CholeskyOuterLoopStep : public fusion::JointVisitor<CholeskyOuterLoopStep>
  {
    typedef boost::fusion::vector<const Model&,
  				  Data&>  ArgsType;
   
    JOINT_VISITOR_INIT(CholeskyOuterLoopStep);


    template<int nvj>
    static void algoSized( const Model& model,
			   Data& data,
			   const int & j,
			   const int & _j);
   

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
  		     JointDataBase<typename JointModel::JointData> &,
  		     const Model& model,
  		     Data& data)
    {
      algoSized<JointModel::NV>(model,data,jmodel.id(),jmodel.idx_v());
    }
  };
  
  
  template<int N,typename M_t>
  void udut( Eigen::MatrixBase<M_t> & M )
  {
    typedef Eigen::Matrix<double,N,N> MatrixNd;
    typedef Eigen::Matrix<double,1,N> VectorNd;
    typedef typename M_t::DiagonalReturnType D_t;
    typedef typename M_t::template TriangularViewReturnType<Eigen::StrictlyUpper>::Type U_t;

    VectorNd tmp;
    D_t D = M.diagonal();
    U_t U = M.template triangularView<Eigen::StrictlyUpper>();

    for(int j=N-1;j>=0;--j )
      {
	typename VectorNd::SegmentReturnType DUt = tmp.tail(N-j-1);
	if( j<N-1 ) DUt = M.row(j).tail(N-j-1).transpose().cwiseProduct( D.tail(N-j-1) );

	D[j] -= M.row(j).tail(N-j-1).dot(DUt);

	for(int i=j-1;i>=0;--i)
	  { U(i,j) -= M.row(i).tail(N-j-1).dot(DUt); U(i,j) /= D[j]; }
      }
  }
  

  template<int NVJ>
  void CholeskyOuterLoopStep::algoSized( const Model& model,
  					 Data& data,
  					 const int & j,
  					 const int & _j)
  {
    typedef typename Model::Index Index;
    Eigen::MatrixXd & M = data.M;
    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    const int NVT = data.nvSubtree[j] - NVJ;

    Eigen::Block<Eigen::MatrixXd> DUt = U.block(NVJ,0,NVT,NVJ);
     DUt  = D.segment(_j+NVJ,NVT).asDiagonal() * U.block( _j,_j+NVJ,NVJ,NVT).transpose();

     Eigen::Block<Eigen::MatrixXd,NVJ,NVJ> Djj = U.template block<NVJ,NVJ>(_j,_j);
     Djj.template triangularView<Eigen::Upper>() 
       = M.template block<NVJ,NVJ>(_j,_j) - U.block( _j,_j+NVJ,NVJ,NVT) * DUt;
     udut<NVJ>(Djj);
     D.template segment<NVJ>(_j) = Djj.diagonal();

    for( Index i=model.parents[j];i>0;i=model.parents[i])
      {
     	const int _i = idx_v(model.joints[i]);
	const int nvi = nv(model.joints[i]);

	for( int k=0;k<nvi;++k )
	  {
	    U.template block<1,NVJ>(_i+k,_j) 
	      = M.template block<1,NVJ>(_i+k,_j) - U.block(_i+k,_j+1,1,NVT) * DUt;
	  }
	// TODO divide by Djj
	assert(false && "TODO divide by Djj");
      }
  }
 
  template<>
  void CholeskyOuterLoopStep::algoSized<1>( const Model& model,
					    Data& data,
					    const int & j,
					    const int & _j)
  {
    typedef typename Model::Index Index;
    Eigen::MatrixXd & M = data.M;
    Eigen::MatrixXd & U = data.U;
    Eigen::VectorXd & D = data.D;
    
    const int NVT = data.nvSubtree[j]-1;
    Eigen::VectorXd::SegmentReturnType DUt = data.tmp.head(NVT);
    if(NVT)
      DUt = U.row(_j).segment(_j+1,NVT).transpose()
	.cwiseProduct(D.segment(_j+1,NVT));

    D[_j] = M(_j,_j) - U.row(_j).segment(_j+1,NVT) * DUt;
    
    for( Index i=model.parents[j];i>0;i=model.parents[i])
      {
	const int _i = idx_v(model.joints[i]);
	const int nvi = nv(model.joints[i]);

	for( int k=0;k<nvi;++k )
	  {
	    U(_i+k,_j) = M(_i+k,_j) - U.row(_i+k).segment(_j+1,NVT).dot(DUt);
	    U(_i+k,_j) /= D[_j];
	  }
      }
  }
 


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

    data.U = Eigen::MatrixXd::Identity(model.nv,model.nv);
    for( int j=model.nbody-1;j>=0;--j )
      {
	CholeskyOuterLoopStep::run(model.joints[j],data.joints[j],
				   CholeskyOuterLoopStep::ArgsType(model,data));
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
