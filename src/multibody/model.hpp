#ifndef __se3_model_hpp__
#define __se3_model_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/force-set.hpp"
#include <iostream>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::SE3);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Inertia);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Force);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Motion);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double,6,Eigen::Dynamic>);

namespace se3
{
  struct Model;
  struct Data;

  class Model
  {
  public:
    typedef int Index;

    int nq;                            // Dimension of the configuration representation
    int nv;                            // Dimension of the velocity vector space
    int nbody;                         // Number of bodies (= number of joints + 1)

    std::vector<Inertia> inertias;     // Spatial inertias of the body <i> in the supporting joint frame <i>
    std::vector<SE3> jointPlacements;  // Placement (SE3) of the input of joint <i> in parent joint output <li>
    JointModelVector joints;           // Model of joint <i>
    std::vector<Index> parents;        // Joint parent of joint <i>, denoted <li> (li==parents[i])
    std::vector<std::string> names;    // name of the body attached to the output of joint <i>

    Motion gravity;                    // Spatial gravity
    static const Eigen::Vector3d gravity981; // Default 3D gravity (=(0,0,9.81))

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
    std::vector<Motion> a;                // Body acceleration
    std::vector<Motion> v;                // Body velocity
    std::vector<Force> f;                 // Body force
    std::vector<SE3> oMi;                 // Body absolute placement (wrt world)
    std::vector<SE3> liMi;                // Body relative placement (wrt parent)
    Eigen::VectorXd tau;                  // Joint forces

    std::vector<Inertia> Ycrb;            // Inertia of the sub-tree composit rigid body
    Eigen::MatrixXd M;                    // Joint Inertia

    typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
    std::vector<Matrix6x> Fcrb;           // Spatial forces set, used in CRBA

    std::vector<Model::Index> lastChild;  // Index of the last child (for CRBA)
    std::vector<int> nvSubtree;           // Dimension of the subtree motion space (for CRBA)

    Eigen::MatrixXd U;                    // Joint Inertia square root (upper triangle)
    Eigen::VectorXd D;                    // Diagonal of UDUT inertia decomposition

    Data( const Model& ref );

    void computeLastChild(const Model& model);
  };

  const Eigen::Vector3d Model::gravity981 (0,0,-9.81);

} // namespace se3



/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
namespace se3
{
  std::ostream& operator<< ( std::ostream & os, const Model& model )
  {
    os << "Nb bodies = " << model.nbody << " (nq="<< model.nq<<",nv="<<model.nv<<")" << std::endl;
    for(int i=0;i<model.nbody;++i)
      {
	os << "  Joint "<<model.names[i] << ": parent=" << model.parents[i] << std::endl;
      }

    return os;
  }

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
    assert( (j.nq()>=0)&&(j.nv()>=0) );

    Index idx = nbody ++;

    joints         .push_back(j.derived()); 
    boost::get<D&>(joints.back()).setIndexes(idx,nq,nv);

    inertias       .push_back(Y);
    parents        .push_back(parent);
    jointPlacements.push_back(placement);
    names          .push_back( (name!="")?name:random(8) );

    nq += j.nq();
    nv += j.nv();
    return idx;
  }
  Model::Index Model::getBodyId( const std::string & name ) const
  {
    Index res = std::find(names.begin(),names.end(),name) - names.begin();
    assert( (res>=0)&&(res<nbody)&&"The body name you asked do not exist" );
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
    ,Ycrb(ref.nbody)
    ,M(ref.nv,ref.nv)
    ,Fcrb(ref.nbody)
    ,lastChild(ref.nbody)
    ,nvSubtree(ref.nbody)
    ,U(ref.nv,ref.nv)
    ,D(ref.nv)
  {
    for(int i=0;i<model.nbody;++i) 
      joints.push_back(CreateJointData::run(model.joints[i]));
    M.fill(NAN);
    for(int i=0;i<ref.nbody;++i ) { Fcrb[i].resize(6,model.nv); Fcrb[i].fill(NAN); }
    computeLastChild(ref);
  }

  void Data::computeLastChild(const Model& model)
  {
    typedef Model::Index Index;
    //lastChild.fill(-1);  TODO use fill algorithm
    for( int i=0;i<model.nbody;++i ) lastChild[i] = -1;


    for( int i=model.nbody-1;i>=0;--i )
      {
	if(lastChild[i] == -1) lastChild[i] = i;
	const Index & parent = model.parents[i];
	lastChild[parent] = std::max(lastChild[i],lastChild[parent]);

	nvSubtree[i] 
	  = Joint_idx_v::run(model.joints[lastChild[i]]) + Joint_nv::run(model.joints[lastChild[i]])
	  - Joint_idx_v::run(model.joints[i]);
      }
  }

} // namespace se3

#endif // ifndef __se3_model_hpp__
