#ifndef __se3_kinematics_hpp__
#define __se3_kinematics_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
  
namespace se3
{
  inline void geometry(const Model & model,
                       Data & data,
                       const Eigen::VectorXd & q);

  inline void kinematics(const Model & model,
			 Data & data,
			 const Eigen::VectorXd & q,
			 const Eigen::VectorXd & v);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct GeometryStep : public fusion::JointVisitor<GeometryStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const int &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (GeometryStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const int & i,
                     const Eigen::VectorXd & q)
    {
      using namespace se3;

      jmodel.calc (jdata.derived (), q);

      const Model::Index & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M ();

      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
    
  };

  inline void
  geometry(const Model & model,
           Data & data,
           const Eigen::VectorXd & q)
  {
    for (size_t i=1; i < (size_t) model.nbody; ++i)
    {
      GeometryStep::run(model.joints[i],
                        data.joints[i],
                        GeometryStep::ArgsType (model,data,i,q)
                        );
    }
  }

  struct KinematicsStep : public fusion::JointVisitor<KinematicsStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const int&,
				   const Eigen::VectorXd &,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(KinematicsStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const int &i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[i];
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
    }

  };

  inline void
  kinematics(const Model & model, Data& data,
	     const Eigen::VectorXd & q,
	     const Eigen::VectorXd & v)
  {
    data.v[0] = Motion::Zero();

    for( int i=1;i<model.nbody;++i )
      {
	KinematicsStep::run(model.joints[i],data.joints[i],
			    KinematicsStep::ArgsType(model,data,i,q,v));
      }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hpp__

