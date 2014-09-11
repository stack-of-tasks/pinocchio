#ifndef __se3_crba2_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
  
namespace se3
{
  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct CrbaForwardStep : public fusion::JointVisitor<CrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(CrbaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;

      const typename JointModel::Index & i = jmodel.id();
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];
    }

  };

  namespace internal 
  {

    template<typename Mat,typename MatRet, int NCOLS>
    struct ForceSetSe3Action
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated
       * with m, and iF, jF are matrices whose columns are forces. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */ 
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iF,
		       Eigen::MatrixBase<MatRet> & jF );
      // {
      //   typename Mat::template ConstNRowsBlockXpr<3>::Type linear  = iF.template topRows<3>();
      //   typename MatRet::template ConstNRowsBlockXpr<3>::Type angular = iF.template bottomRows<3>();
      
      //   jF.template topRows   <3>().noalias() = m.rotation()*linear;
      //   jF.template bottomRows<3>().noalias()
      // 	= skew(m.translation())*jF.template topRows<3>() +
      //      m.rotation()*angular;
      // }
      
    };

    template<typename Mat,typename MatRet>
    struct ForceSetSe3Action<Mat,MatRet,1>
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated with m,
       * and iF, jF are vectors. */
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iF,
		       Eigen::MatrixBase<MatRet> & jF )
      { 
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
	Eigen::VectorBlock<const Mat,3> linear = iF.template head<3>();
	Eigen::VectorBlock<const Mat,3> angular = iF.template tail<3>();
	
	jF.template head <3>() = m.rotation()*linear;
	jF.template tail <3>() = (m.translation().cross(jF.template head<3>())
				  + m.rotation()*angular);
      }
    };

    template<typename Mat,typename MatRet>
    static void se3Action( const SE3 & m, 
			   const Eigen::MatrixBase<Mat> & iF,
			   Eigen::MatrixBase<MatRet> & jF )
    {
      ForceSetSe3Action<Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }

    template<typename Mat,typename MatRet,int NCOLS>
    void ForceSetSe3Action<Mat,MatRet,NCOLS>::
    run( const SE3 & m, 
	 const Eigen::MatrixBase<Mat> & iF,
	 Eigen::MatrixBase<MatRet> & jF )
    {
      for(int col=0;col<jF.cols();++col) 
	{
	  typename MatRet::ColXpr jFc = jF.col(col);
	  se3Action(m,iF.col(col),jFc);
	}
    }
    
  } // namespace internal

  struct CrbaBackwardStep : public fusion::JointVisitor<CrbaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&>  ArgsType;
    
    JOINT_VISITOR_INIT(CrbaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0 
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      const Model::Index & i = jmodel.id();

      /* F[1:6,i] = Y*S */
      data.Fcrb[i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]) 
	= jdata.S().transpose()*data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);

      const Model::Index & parent   = model.parents[i];
      if(parent>0) 
	{
	  /*   Yli += liXi Yi */
 	  data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

	  /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
	  Eigen::Block<typename Data::Matrix6x> jF
	    = data.Fcrb[parent].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
 	  internal::se3Action(data.liMi[i],
			      data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]),
			      jF);
	}
      
      // std::cout << "iYi = " << (Inertia::Matrix6)data.Ycrb[i] << std::endl;
      // std::cout << "iSi = " << ConstraintXd(jdata.S()).matrix() << std::endl;
      // std::cout << "liFi = " << jdata.F() << std::endl;
      // std::cout << "M = " <<  data.M << std::endl;
    }
  };

  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q)
  {
    for( int i=1;i<model.nbody;++i )
      {
	CrbaForwardStep::run(model.joints[i],data.joints[i],
			     CrbaForwardStep::ArgsType(model,data,q));
      }
    
    for( int i=model.nbody-1;i>0;--i )
      {
	CrbaBackwardStep::run(model.joints[i],data.joints[i],
			      CrbaBackwardStep::ArgsType(model,data));
      }

    return data.M;
  }
} // namespace se3

#endif // ifndef __se3_crba_hpp__

