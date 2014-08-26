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
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
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
       * F[:,i] = Yi*Si
       * M[i,i:subtree] = Si'*F[:,i:substree]
       * if li>0:
       *    Yli += liMi.act(Yi)
       *    F[:,i:subtree] = liMi.act(F[:,i:subtree])
       */

      const Model::Index & parent   = model.parents[i];
      const int &          nsubtree = data.nvSubtree[i];

      data.Fcrb.block(jmodel.idx_v(),JointModel::nv) = data.Ycrb[i] * jdata.S();

      data.M.block(jmodel.idx_v(),jmodel.idx_v(),JointModel::nv,nsubtree)
       	= jdata.S().transpose() * data.Fcrb.block(jmodel.idx_v(),nsubtree);

      // std::cout << "*** joint " << i << std::endl;
      // std::cout << "iYi = " << (Inertia::Matrix6)data.Ycrb[i] << std::endl;
      // std::cout << "iSi = " << ConstraintXd(jdata.S()).matrix() << std::endl;
      // std::cout << "iF = " << data.Fcrb.matrix() << std::endl;
      // std::cout << "M = " <<  data.M << std::endl;

      if( parent>0 )
	{
	  data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
	  data.Fcrb.block(jmodel.idx_v(), nsubtree) =
	    data.liMi[i].act( data.Fcrb.block(jmodel.idx_v(), nsubtree) );
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

