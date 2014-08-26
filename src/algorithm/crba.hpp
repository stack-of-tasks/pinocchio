#ifndef __se3_crba_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
  
namespace se3
{
  inline const Eigen::VectorXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct CrbaForwardStep : public fusion::JointVisitor<CrbaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
			    se3::Data&,
			    const int&,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(CrbaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		     se3::JointDataBase<typename JointModel::JointData> & jdata,
		     const se3::Model& model,
		     se3::Data& data,
		     const int &i,
		     const Eigen::VectorXd & q)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q);
      
      const Model::Index & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];
      
      data.Ycrb[i] = model.inertias[i];
    }

  };

  struct CrbaBackwardStep : public fusion::JointVisitor<CrbaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&,
				  const int &>  ArgsType;
    
    JOINT_VISITOR_INIT(CrbaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data,
		     int i)
    {
      /*
       * F = Yi*Si
       * for j<i 
       *    F = ljXj.act(F)
       *    M.block(j,i,nvj,nvi) = Sj'*F
       */
      std::cout << "*** joint " << i << std::endl;

      Model::Index parent = model.parents[i];
      if( parent>0 )
	data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

      //std::cout << "liMi = " << (SE3::Matrix4)data.liMi[i] << std::endl;

      Eigen::Matrix<double,6,JointModel::nv> F;
      ConstraintXd S = jdata.S();
      F = data.Ycrb[i].toMatrix() * S.matrix() ;

      std::cout << "*** joint " << i << std::endl;
      std::cout << "iYi = " << (Inertia::Matrix6)data.Ycrb[i] << std::endl;
      std::cout << "iSi = " << S.matrix() << std::endl;
      std::cout << "iFi = " << F << std::endl;

      data.M.block<JointModel::nv,JointModel::nv>(jmodel.idx_v(),jmodel.idx_v())
	= S.matrix().transpose() * F;

      SE3::Matrix6 ljXj = data.liMi[i];
      while(parent>0)
	{
	  JointDataGeneric jdataparent( data.joints[parent] );
	  JointModelGeneric jmodelparent( model.joints[parent] );

	  F = ljXj.inverse().transpose()*F; // equivalent to ljF = ljMj.act(jF)
	  const ConstraintXd::DenseBase & S = jdataparent.S.matrix();
	  std::cout << "jFi = " << F <<std::endl;
	  std::cout << "jS = " << S <<std::endl;

	  data.M.block(jmodelparent.idx_v(),jmodel.idx_v(),S.cols(),JointModel::nv)
	    = S.transpose() * F;

	  std::cout << "\t\t on parent #i,j = " << i<<","<<parent << "   ... compute block "
		    << jmodelparent.idx_v() << ":" 
		    << jmodelparent.idx_v() + S.cols() << " x "
		    << jmodel.idx_v() << ":"
		    << jmodel.idx_v() + JointModel::nv << std::endl;
	  std::cout << "jFi = " << F << std::endl;
	  std::cout << "jSj = " << S << std::endl;

	  ljXj = data.liMi[parent];
	  parent = model.parents[parent];
	}

    }
  };

  inline const Eigen::MatrixXd&
  crba(const Model & model, Data& data,
       const Eigen::VectorXd & q)
  {
    for( int i=1;i<model.nbody;++i )
      {
	CrbaForwardStep::run(model.joints[i],data.joints[i],
			     CrbaForwardStep::ArgsType(model,data,i,q));
      }
    
    for( int i=model.nbody-1;i>0;--i )
      {
	CrbaBackwardStep::run(model.joints[i],data.joints[i],
			      CrbaBackwardStep::ArgsType(model,data,i));
      }

    return data.M;
  }
} // namespace se3

#endif // ifndef __se3_crba_hpp__

