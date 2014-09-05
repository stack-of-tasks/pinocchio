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
  template<typename JointModel,typename D>
  void udut( Eigen::MatrixBase<D> & U,
	     const JointModel& )
  {
    /* TODO */
  }


  // struct CholeskyInnerStep : public fusion::JointVisitor<CholeskyInnerStep>
  // {
  //   typedef boost::fusion::vector<const Model&,
  // 				  Data&,
  // 				  const int &,
  // 				  const int &>  ArgsType;
    
  //   JOINT_VISITOR_INIT(CholeskyOuterStep);
 
  //   template<typename JointModel>
  //   static void algo(const JointModelBase<JointModel> & jmodel,
  // 		     JointDataBase<typename JointModel::JointData> & ,
  // 		     const Model& model,
  // 		     Data& data,
  // 		     int j)
  //   {
      
  //   }
  // };
  
  // struct CholeskyOuterStep : public fusion::JointVisitor<CholeskyOuterStep>
  // {
  //   typedef boost::fusion::vector<const Model&,
  // 				  Data&,
  // 				  const int &>  ArgsType;
    
  //   JOINT_VISITOR_INIT(CholeskyOuterStep);

  //   template<typename JointModel>
  //   static void algo(const JointModelBase<JointModel> & jmodel,
  // 		     JointDataBase<typename JointModel::JointData> & ,
  // 		     const Model& model,
  // 		     Data& data,
  // 		     int j)
  //   {
  //     typename Model::Index parent = jmodel.parents[j];
  //     while( parent>0 )
  // 	{
	  
  // 	}

  //     const int 
  // 	idx = jmodel.idx_v(),
  // 	nv = JointModel::nv,
  // 	idx_tree = idx+nv,
  // 	nv_tree = data.nvSubtree[j];
      
  //     data.U.template block<nv,nv>(j,j)
  // 	-= data.U.block(idx,idx_tree,nv,nv_tree)
  // 	* data.D.segment(idx_tree,nv_tree).asDiagonal()
  // 	* data.U.block(idx,idx_tree,nv,nv_tree).transpose();
      
  //     Eigen::Block<Eigen::MatrixXd,nv,nv> Ujj = data.U.template block<nv,nv>(j,j);
  //     udut(Ujj,jmodel);
  //   }
  // };

 
  template<typename JointModel_k,typename JointModel_i>
  struct CholeskyLoopJStep : public fusion::JointVisitor< CholeskyLoopJStep<JointModel_k,JointModel_i> >
  {
    typedef boost::fusion::vector<const JointModelBase<JointModel_k>&,
				  const JointModelBase<JointModel_i>&,
				  const Model&,
				  Data&,
				  const int &,
				  const int &,
				  const double &>  ArgsType;
    
    CholeskyLoopJStep( JointDataVariant & jdata,ArgsType args ) : jdata(jdata),args(args) {}
    using fusion::JointVisitor< CholeskyLoopJStep<JointModel_k,JointModel_i>  >::run;
    JointDataVariant & jdata;			
    ArgsType args;

    template<typename JointModel_j>
    static void algo(const JointModelBase<JointModel_j> & jj_model,
		     JointDataBase<typename JointModel_j::JointData> &,
		     const JointModelBase<JointModel_k> & jk_model,
		     const JointModelBase<JointModel_i> & ji_model,
		     const Model& model,
		     Data& data,
		     const int & k,
		     const int & i,
		     const int & j)
    {
    }
  };
  
 
  template<typename JointModel_k>
  struct CholeskyLoopIStep : public fusion::JointVisitor< CholeskyLoopIStep<JointModel_k> >
  {
    typedef boost::fusion::vector<const JointModelBase<JointModel_k>&,
  				  const Model&,
  				  Data&,
  				  const int &,
  				  const int &>  ArgsType;
    
    //JOINT_VISITOR_INIT(CholeskyLoopIStep);
    CholeskyLoopIStep( JointDataVariant & jdata,ArgsType args ) : jdata(jdata),args(args) {}
    using fusion::JointVisitor< CholeskyLoopIStep<JointModel_k>  >::run;				       
    JointDataVariant & jdata;			
    ArgsType args;


    template<typename JointModel_i>
    static void algo(const JointModelBase<JointModel_i> & ji_model,
  		     JointDataBase<typename JointModel_i::JointData> &,
  		     const JointModelBase<JointModel_k> & jk_model,
  		     const Model& model,
  		     Data& data,
  		     const int & k,
  		     const int & i)
    {
      
      double a = data.M( jk_model.idx_v(),ji_model.idx_v() )/data.M(k,k);
      typedef typename Model::Index Index;
      for( Index j=i;j>0;j=model.parents[j])
	{
       	  CholeskyLoopJStep<JointModel_k,JointModel_i>
	    ::run(model.joints[j],data.joints[j],
		  typename CholeskyLoopJStep<JointModel_k,JointModel_i>
		  ::ArgsType(jk_model,ji_model,model,data,k,i,a) );
       	}
    }
  };
  
  struct CholeskyLoopKStep : public fusion::JointVisitor<CholeskyLoopKStep>
  {
    typedef boost::fusion::vector<const Model&,
  				  Data&,
  				  const int &>  ArgsType;
    
    JOINT_VISITOR_INIT(CholeskyLoopKStep);
 
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jk_model,
  		     JointDataBase<typename JointModel::JointData> & jk_data,
  		     const Model& model,
  		     Data& data,
  		     const int & k )
    {
      typedef typename Model::Index Index;
      for( Index i=model.parents[k];i>0;i=model.parents[i])
  	{
  	  CholeskyLoopIStep<JointModel>
  	    ::run(model.joints[i],data.joints[i],
  	     	  typename CholeskyLoopIStep<JointModel>
  		  ::ArgsType(jk_model,model,data,k,i) );
  	}
    }
  };
  
  /*
   * U=zeros(n,n); D=zeros(n,1);
   * for j=n:-1:1
   *     for k=n:-1:j+1
   *         subtree = k+1:n;
   *         U(j,k) = inv(D(k)) * (A(j,k) - U(k,subtree)*diag(D(subtree))*U(j,subtree)');
   *     end
   *     subtree = j+1:n;
   *     D(j) = A(j,j) - U(j,subtree)*diag(D(subtree))*U(j,subtree)';
   * end
   * U = U + diag(ones(n,1));
   */
  inline const Eigen::MatrixXd&
  cholesky(const Model &           model, 
	   Data&                   data )
  {
    data.U = data.M;
    for( int j=model.nbody-1;j>=0;--j )
      {
	CholeskyLoopKStep::run(model.joints[j],data.joints[j],
	 		       CholeskyLoopKStep::ArgsType(model,data,j));

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
