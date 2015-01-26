#ifndef __se3_crba_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/act-on-set.hpp"

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
 	  forceSet::se3Action(data.liMi[i],
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

