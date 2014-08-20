#ifndef __se3_model_hpp__
#define __se3_model_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/multibody/joint.hpp"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion);

namespace se3
{
  struct Model;
  struct Data;

  class Model
  {
  public:
    typedef int Index;

    int nq;
    int nv;
    int nbody;

    std::vector<Inertia> inertias;
    std::vector<SE3> jointPlacements;
    JointModelVector joints;
    //std::vector<JointModelRX> joints;
    std::vector<Index> parents;
    std::vector<std::string> names;

    Motion gravity;
    static const Eigen::Vector3d gravity981;

    Model()
      : nq(0)
      , nv(0)
      , nbody(1)
      , inertias(1)
      , jointPlacements(1)
      , joints(1)
      , parents(1)
      , names(1)
      , gravity( gravity981,Eigen::Vector3d::Zero() )
    {
      names[0] = "universe";
    }
    template<typename D>
    Index addBody( Index parent,const JointModelBase<D> & j,const SE3 & placement,
		   const Inertia & Y,const std::string & name = "" );
    Index getBodyId( const std::string & name ) const;
    const std::string& getBodyName( Index index ) const;

  };

  class Data
  {
  public:
    
    const Model& model;
    JointDataVector joints;
    //std::vector<JointDataRX> joints;
    std::vector<Motion> a;                // Body acceleration
    std::vector<Motion> v;                // Body velocity
    std::vector<Force> f;                 // Body force
    std::vector<SE3> oMi;                 // Body absolute placement (wrt world)
    std::vector<SE3> liMi;                // Body relative placement (wrt parent)
    Eigen::VectorXd tau;                  // Joint forces

    Data( const Model& ref );


  };

  const Eigen::Vector3d Model::gravity981 (0,0,9.81);

} // namespace se3


/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
namespace se3
{
  
  std::string random(const int len)
  {
    std::string res;
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i=0; i<len;++i)
      res += alphanum[std::rand() % (sizeof(alphanum) - 1)];
    return res;
}

  template<typename D>
  Model::Index Model::addBody( Index parent,const JointModelBase<D> & j,const SE3 & placement,
			       const Inertia & Y,const std::string & name )
  {
    assert( (nbody==(int)joints.size())&&(nbody==(int)inertias.size())
	    &&(nbody==(int)parents.size())&&(nbody==(int)jointPlacements.size()) );
    assert( (j.nq>=0)&&(j.nv>=0) );

    Index idx = nbody ++;

    joints         .push_back(j.derived()); 
    boost::get<D&>(joints.back()).setIndexes(nq,nv);

    inertias       .push_back(Y);
    parents        .push_back(parent);
    jointPlacements.push_back(placement);
    names          .push_back( (name!="")?name:random(8) );

    nq += j.nq;
    nv += j.nv;
    return idx;
  }
  Model::Index Model::getBodyId( const std::string & name ) const
  {
    Index res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res>=0)&&(res<nbody) );
    return res;
  }
  
  const std::string& Model::getBodyName( Model::Index index ) const
  {
    assert( (index>=0)&&(index<nbody) );
    return names[index];
  }  

  Data::Data( const Model& ref )
    :model(ref)
    ,joints(0)
    ,a(ref.nbody)
    ,v(ref.nbody)
    ,f(ref.nbody)
    ,oMi(ref.nbody)
    ,liMi(ref.nbody)
    ,tau(ref.nv)
  {
    for(int i=0;i<model.nbody;++i) 
      //joints.push_back(model.joints[i].createData());
      joints.push_back(CreateJointData::run(model.joints[i]));
  }


} // namespace se3

#endif // ifndef __se3_model_hpp__
