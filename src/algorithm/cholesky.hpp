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
  struct CholeskyLoopKStep : public fusion::JointVisitor<CholeskyLoopKStep>
  {
    typedef boost::fusion::vector<const Model&,
  				  Data&,
  				  const int &>  ArgsType;
   
    JOINT_VISITOR_INIT(CholeskyLoopKStep);
 
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jk_model,
  		     JointDataBase<typename JointModel::JointData> &,
  		     const Model& model,
  		     Data& data,
  		     const int & k )
    {
      typedef typename Model::Index Index;
      Eigen::MatrixXd & U = data.U;
      const int & _k = jk_model.idx_v();

      for( Index i=model.parents[k];i>0;i=model.parents[i])
       	{
       	  const int _i = idx_v(model.joints[i]);
	  
       	  const double a = 1/U(_k,_k) * U(_i,_k);
       	  U(_i,_i) -= a * U(_i,_k);
	  
       	  for( Index j=model.parents[i];j>0;j=model.parents[j])
       	    {
       	      const int _j = idx_v(model.joints[j]);
       	      U(_j,_i) -= a * U(_j,_k);
       	    }
	  
       	  U(_i,_k) = a;
       	}
    }
  };
  
  inline const Eigen::MatrixXd&
  cholesky(const Model &           model, 
	   Data&                   data )
  {
    /* for k=n:-1:1
     *   i=parent(k);
     *   while i>0
     *       a = M(i,k) / M(k,k);
     *       M(i,i) = M(i,i) - a * M(i,k);
     *       j = parent(i);
     *       while j>0
     *           M(j,i) = M(j,i) - a * M(j,k);
     *          j=parent(j);
     *       end
     *       M(i,k) = a;
     *       i=parent(i);
     *   end
     * end
     */
    data.U = data.M;
    for( int j=model.nbody-1;j>=0;--j )
      {
	CholeskyLoopKStep::run(model.joints[j],data.joints[j],
	 		       CholeskyLoopKStep::ArgsType(model,data,j));

       }
    data.D = data.U.diagonal();
    data.U.diagonal().fill(1);

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
