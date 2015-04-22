#ifndef __se3_rnea_hpp__
#define __se3_rnea_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct RneaForwardStep : public fusion::JointVisitor<RneaForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
			    se3::Data&,
			    const int&,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(RneaForwardStep);

    template<typename JointModel>
    static int algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const int &i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v,
		    const Eigen::VectorXd & a)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[(std::size_t)i];
      data.liMi[(std::size_t)i] = model.jointPlacements[(std::size_t)i]*jdata.M();
      
      data.v[(std::size_t)i] = jdata.v();
      if(parent>0) data.v[(std::size_t)i] += data.liMi[(std::size_t)i].actInv(data.v[(std::size_t)parent]);
      
      data.a[(std::size_t)i]  = jdata.S()*jmodel.jointMotion(a) + jdata.c() + (data.v[(std::size_t)i] ^ jdata.v()) ; 
      data.a[(std::size_t)i] += data.liMi[(std::size_t)i].actInv(data.a[(std::size_t)parent]);
      
      data.f[(std::size_t)i] = model.inertias[(std::size_t)i]*data.a[(std::size_t)i] + model.inertias[(std::size_t)i].vxiv(data.v[(std::size_t)i]); // -f_ext
      return 0;
    }

  };

  struct RneaBackwardStep : public fusion::JointVisitor<RneaBackwardStep>
  {
    typedef boost::fusion::vector<const Model&,
				  Data&,
				  const int &>  ArgsType;
    
    JOINT_VISITOR_INIT(RneaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model& model,
		     Data& data,
		     int i)
    {
      const Model::Index & parent  = model.parents[(std::size_t)i];      
      jmodel.jointForce(data.tau)  = jdata.S().transpose()*data.f[(std::size_t)i];
      if(parent>0) data.f[(std::size_t)parent] += data.liMi[(std::size_t)i].act(data.f[(std::size_t)i]);
    }
  };

  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a)
  {
    data.v[0] = Motion::Zero();
    data.a[0] = -model.gravity;

    for( int i=1;i<model.nbody;++i )
      {
	RneaForwardStep::run(model.joints[(std::size_t)i],data.joints[(std::size_t)i],
			     RneaForwardStep::ArgsType(model,data,i,q,v,a));
      }
    
    for( int i=model.nbody-1;i>0;--i )
      {
	RneaBackwardStep::run(model.joints[(std::size_t)i],data.joints[(std::size_t)i],
	 		      RneaBackwardStep::ArgsType(model,data,i));
      }

    return data.tau;
  }
} // namespace se3

#endif // ifndef __se3_rnea_hpp__

