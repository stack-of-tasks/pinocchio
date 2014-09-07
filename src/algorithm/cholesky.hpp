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
 
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
  		     JointDataBase<typename JointModel::JointData> &,
  		     const Model& model,
  		     Data& data)
    {
      typedef typename Model::Index Index;
      Eigen::MatrixXd & M = data.M;
      Eigen::MatrixXd & U = data.U;
      Eigen::VectorXd & D = data.D;
      const int j = jmodel.id();
      enum { _j = JointModel::NV };

      D[_j] = (M(_j,_j) 
	       - U.row(_j).segment(_j+1,_j+data.nvSubtree[j])  
	       * D.segment(_j+1,_j+data.nvSubtree[j]).asDiagonal()
	       * U.row(_j).segment(_j+1,_j+data.nvSubtree[j]).transpose());

      for( Index i=model.parents[j];i>0;i=model.parents[i])
       	{
       	  const int _i = idx_v(model.joints[i]);
	  U(_i,_j) = (M(_i,_j)
		      - U.row(_i).segment(_j+1,_j+data.nvSubtree[j])  
		      * D.segment(_j+1,_j+data.nvSubtree[j]).asDiagonal()
		      * U.row(_j).segment(_j+1,_j+data.nvSubtree[j]).transpose()) / D[_j];
       	}
    }
  };
  
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

    data.U = Eigen::MatrixXd(model.nbody,model.nbody);
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
