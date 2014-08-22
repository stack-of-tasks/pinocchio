#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#define     BOOST_FUSION_INVOKE_MAX_ARITY 10
#include <boost/fusion/include/sequence.hpp>
#include <boost/fusion/include/make_vector.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/view/joint_view.hpp>
#include <boost/fusion/include/joint_view.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/container.hpp>


namespace boost {
  namespace fusion {
    template<typename T,typename V>
    typename result_of::push_front<V const, T>::type
    append(T const& t,V const& v) { return push_front(v,t); }

    template<typename T1,typename T2,typename V>
    typename result_of::push_front<typename result_of::push_front<V const, T2>::type const, T1>::type
    append2(T1 const& t1,T2 const& t2,V const& v) { return push_front(push_front(v,t2),t1); }
  }
}
  

namespace se3
{

  namespace bf = boost::fusion;
  

  struct RneaForwardStep : public boost::static_visitor<>
  {

    typedef bf::vector< const se3::Model& ,
			se3::Data& ,
			int,
			const Eigen::VectorXd &,
			const Eigen::VectorXd &,
			const Eigen::VectorXd &
			> Args;
    Args args;
    JointDataVariant & jdata;

    RneaForwardStep( JointDataVariant & jdata,Args args ) : args(args),jdata(jdata) {}
    
    template<typename D>
    void operator() (const JointModelBase<D> & jmodel) const
    {
      bf::invoke(&RneaForwardStep::algo<D>,
		 bf::append2(jmodel,boost::ref(boost::get<typename D::JointData&>(jdata)),args));
    }

    static void run(const JointModelVariant & jmodel,
		    JointDataVariant & jdata,
		    Args args)
    {
      return boost::apply_visitor( RneaForwardStep(jdata,args),jmodel );
    }
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
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
      
      const Model::Index & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0) data.oMi[i] = data.oMi[parent]*data.liMi[i];
      else         data.oMi[i] = data.liMi[i];
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      jmodel.jointMotion(a);
      data.a[i] =  jdata.S()*jmodel.jointMotion(a) + jdata.c() + (data.v[i] ^ jdata.v()) ; 
      if(parent>0) data.a[i] += data.liMi[i].actInv(data.a[parent]);
      
      data.f[i] = model.inertias[i]*data.a[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };




struct RneaBackwardStep : public boost::static_visitor<>
{
  typedef bf::vector<const Model& ,
		     Data& ,
		     const int &>  Args;
  JointDataVariant& jdata;
  Args args;

  RneaBackwardStep( JointDataVariant& jdata, Args args)
    : jdata(jdata),args(args) {}

  template <typename D>
  void operator()(const JointModelBase<D> & jmodel) const
  {
    bf::invoke(&RneaBackwardStep::algo<D>,
	       bf::append2(jmodel,boost::ref(boost::get<typename D::JointData&>(jdata)),args));
  }

  static void run( const JointModelVariant & jmodel, 
		   JointDataVariant & jdata,
		   Args args)
  {  boost::apply_visitor( RneaBackwardStep(jdata,args),jmodel ); }

  template<typename JointModel>
  static void algo(const JointModelBase<JointModel> & jmodel,
	    JointDataBase<typename JointModel::JointData> & jdata,
	    const Model& model,
	    Data& data,
	    int i)
  {
    const Model::Index & parent  = model.parents[i];      
    jmodel.jointForce(data.tau)  = jdata.S().transpose()*data.f[i];
    if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
  }
};

} // namespace se3

//#define __SSE3__
#include <fenv.h>
#ifdef __SSE3__
#include <pmmintrin.h>
#endif

int main()
{
#ifdef __SSE3__
_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif


  using namespace Eigen;
  using namespace se3;


  se3::Model model;
  model.addBody(model.getBodyId("universe"),JointModelFreeFlyer(),SE3::Random(),Inertia::Random(),"root");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg1");
  model.addBody(model.getBodyId("rleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg2");
  model.addBody(model.getBodyId("rleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg3");
  model.addBody(model.getBodyId("rleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg4");
  model.addBody(model.getBodyId("rleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg5");
  model.addBody(model.getBodyId("rleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rleg6");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg1");
  model.addBody(model.getBodyId("lleg1"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg2");
  model.addBody(model.getBodyId("lleg2"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg3");
  model.addBody(model.getBodyId("lleg3"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg4");
  model.addBody(model.getBodyId("lleg4"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg5");
  model.addBody(model.getBodyId("lleg5"),JointModelRX(),SE3::Random(),Inertia::Random(),"lleg6");

  model.addBody(model.getBodyId("root"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso1");
  model.addBody(model.getBodyId("torso1"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso2");
  //model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"torso3");
  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck1");
  model.addBody(model.getBodyId("neck1"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck2");
  //model.addBody(model.getBodyId("neck2"),JointModelRX(),SE3::Random(),Inertia::Random(),"neck3");

  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm1");
  model.addBody(model.getBodyId("rarm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm2");
  model.addBody(model.getBodyId("rarm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm3");
  model.addBody(model.getBodyId("rarm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm4");
  model.addBody(model.getBodyId("rarm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm5");
  model.addBody(model.getBodyId("rarm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"rarm6");
  model.addBody(model.getBodyId("rarm6"),JointModelRX(),SE3::Random(),Inertia::Random(),"rgrip");

  model.addBody(model.getBodyId("torso2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm1");
  model.addBody(model.getBodyId("larm1"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm2");
  model.addBody(model.getBodyId("larm2"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm3");
  model.addBody(model.getBodyId("larm3"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm4");
  model.addBody(model.getBodyId("larm4"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm5");
  model.addBody(model.getBodyId("larm5"),JointModelRX(),SE3::Random(),Inertia::Random(),"larm6");
  model.addBody(model.getBodyId("larm6"),JointModelRX(),SE3::Random(),Inertia::Random(),"lgrip");

  se3::Data data(model);

  VectorXd q = VectorXd::Random(model.nq);
  VectorXd v = VectorXd::Random(model.nv);
  VectorXd a = VectorXd::Random(model.nv);

  data.v[0] = Motion::Zero();
  data.a[0] = -model.gravity;
 
  StackTicToc timer(StackTicToc::US); timer.tic();
  SMOOTH(1000)
    {
  for( int i=1;i<model.nbody;++i )
    {
      RneaForwardStep::run(model.joints[i],data.joints[i],
			   RneaForwardStep::Args(model,data,i,q,v,a));
    }

  for( int i=model.nbody-1;i>0;--i )
    {
      RneaBackwardStep::run(model.joints[i],data.joints[i],
			    RneaBackwardStep::Args(model,data,i));
    }
    }
  timer.toc(std::cout,1000);

  return 0;
}
