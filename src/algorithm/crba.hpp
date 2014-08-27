#ifndef __se3_crba_hpp__
#define __se3_crba_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include <iostream>
  
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

  namespace internal 
  {
    template<typename D>
    static Eigen::Matrix<double,6,D::ColsAtCompileTime>
    act( const SE3 & m, const Eigen::MatrixBase<D> & iF )
    {
      Eigen::Matrix<double,6,D::ColsAtCompileTime> jF(iF.rows(),iF.cols());
      
      typename D::template ConstNRowsBlockXpr<3>::Type linear  = iF.template topRows<3>();
      typename D::template ConstNRowsBlockXpr<3>::Type angular = iF.template bottomRows<3>();
      
      jF.template topRows   <3>() = m.rotation()*linear;
      jF.template bottomRows<3>() = skew(m.translation())*jF.template topRows<3>()
	+m.rotation()*angular;
      return jF;
    }
    
    typedef Eigen::Matrix<double,6,1> Vector6;
    static Vector6 act( const SE3 & m, const Eigen::MatrixBase<Vector6> & iF )
    {
      Vector6 jF;
      
      Eigen::VectorBlock<const Vector6,3> linear = iF.head<3>();
      Eigen::VectorBlock<const Vector6,3> angular = iF.tail<3>();
      
      jF.head <3>() = m.rotation()*linear;
      jF.tail <3>() = m.translation().cross(jF.head<3>()) + m.rotation()*angular;
      return jF;
    }
  } // namespace internal

  template<typename D>
  struct CrbaInternalBackwardStep : public fusion::JointVisitor<CrbaInternalBackwardStep<D> >
  {
    typedef boost::fusion::vector<const Model&,
				  Data&,
				  const Eigen::MatrixBase<D>&,
				  const int &,
				  const int &>  ArgsType;
    
    CrbaInternalBackwardStep( JointDataVariant & jdata,ArgsType args ) : jdata(jdata),args(args) {}
    using fusion::JointVisitor< CrbaInternalBackwardStep<D> >::run;				       
    JointDataVariant & jdata;
    ArgsType args;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
		     JointDataBase<typename JointModel::JointData> & jdata,
		     const Model&,
		     Data& data,
		     const Eigen::MatrixBase<D> & F,
		     const int & idx_v, const int & nv)
    {
      data.M.block(jmodel.idx_v(),idx_v,JointModel::nv,nv) = jdata.S().transpose()*F;
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
       * F = Y*S
       * M[i,i] = S'*F
       * for j<i
       *   F = jXi*F
       *   M[j,i] = S'*F
       * if li>0 Yli += liXi Yi
       */

      Model::Index  parent   = model.parents[i];

      jdata.F() = data.Ycrb[i] * jdata.S();
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),JointModel::nv,JointModel::nv)
	= jdata.S().transpose()*jdata.F();

      // std::cout << "*** joint " << i << std::endl;
      // std::cout << "iFi = " << jdata.F() << std::endl;

      if(parent>0) 
	{
	  data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);
	  jdata.F() = internal::act(data.liMi[i],jdata.F());
	}
      
      // std::cout << "iYi = " << (Inertia::Matrix6)data.Ycrb[i] << std::endl;
      // std::cout << "iSi = " << ConstraintXd(jdata.S()).matrix() << std::endl;
      // std::cout << "liFi = " << jdata.F() << std::endl;
      
      while(parent>0)
	{
	  CrbaInternalBackwardStep<typename JointModel::F_t>
	    ::run( model.joints[parent],
	  	   data.joints[parent],
	  	   typename CrbaInternalBackwardStep<typename JointModel::F_t>
	  	   ::ArgsType(model,data,jdata.F(),jmodel.idx_v(),JointModel::nv) );

	  jdata.F() = internal::act(data.liMi[parent],jdata.F());
	  parent = model.parents[parent];

	  // std::cout << "\tj =" << parent << std::endl;
	  // std::cout << "\tjFi = " << jdata.F() << std::endl;
	  // std::cout << "\tM = " << data.M.block(0,jmodel.idx_v(),model.nv,JointModel::nv) << std::endl;
	}

      
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

