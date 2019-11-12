//
// Copyright (c) 2016-2019 CNRS, INRIA
//

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;
using namespace Eigen;

template<bool local>
Data::Matrix6x finiteDiffJacobian(const Model & model, Data & data, const Eigen::VectorXd & q, const Model::JointIndex joint_id)
{
  Data::Matrix6x res(6,model.nv); res.setZero();
  VectorXd q_integrate (model.nq);
  VectorXd v_integrate (model.nv); v_integrate.setZero();
  
  forwardKinematics(model,data,q);
  const SE3 oMi_ref = data.oMi[joint_id];
  
  double eps = 1e-8;
  for(int k=0; k<model.nv; ++k)
  {
    // Integrate along kth direction
    v_integrate[k] = eps;
    q_integrate = integrate(model,q,v_integrate);
    
    forwardKinematics(model,data,q_integrate);
    const SE3 & oMi = data.oMi[joint_id];
    
    if (local)
      res.col(k) = log6(oMi_ref.inverse()*oMi).toVector();
    else
      res.col(k) = oMi_ref.act(log6(oMi_ref.inverse()*oMi)).toVector();
    
    res.col(k) /= eps;
    
    v_integrate[k] = 0.;
  }
  
  return res;
}

template<typename Matrix>
void filterValue(MatrixBase<Matrix> & mat, typename Matrix::Scalar value)
{
  for(int k = 0; k < mat.size(); ++k)
    mat.derived().data()[k] =  math::fabs(mat.derived().data()[k]) <= value?0:mat.derived().data()[k];
}

template<typename JointModel_> struct init;

template<typename JointModel_>
struct init
{
  static JointModel_ run()
  {
    JointModel_ jmodel;
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options>
struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> >
{
  typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
  
  static JointModel run()
  {
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    JointModel jmodel((JointModelRX()));
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename Scalar, int Options, template<typename,int> class JointCollection>
struct init<pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> >
{
  typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> JointModel;
  
  static JointModel run()
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
    JointModel jmodel((JointModelRX()));
    jmodel.addJoint(JointModelRY());
    
    jmodel.setIndexes(0,0,0);
    return jmodel;
  }
};

template<typename JointModel_>
struct init<pinocchio::JointModelMimic<JointModel_> >
{
  typedef pinocchio::JointModelMimic<JointModel_> JointModel;
  
  static JointModel run()
  {
    JointModel_ jmodel_ref = init<JointModel_>::run();
    
    JointModel jmodel(jmodel_ref,1.,0.);
    jmodel.setIndexes(0,0,0);
    
    return jmodel;
  }
};

struct FiniteDiffJoint
{
  void operator()(JointModelComposite & /*jmodel*/) const
  {}
  
  template<typename JointModel>
  void operator()(JointModelBase<JointModel> & /*jmodel*/) const
  {
    typedef typename JointModel::ConfigVector_t CV;
    typedef typename JointModel::TangentVector_t TV;
    typedef typename LieGroup<JointModel>::type LieGroupType;
    
    JointModel jmodel = init<JointModel>::run();
    std::cout << "name: " << jmodel.classname() << std::endl;
    
    typename JointModel::JointDataDerived jdata_ = jmodel.createData();
    typedef JointDataBase<typename JointModel::JointDataDerived> DataBaseType;
    DataBaseType & jdata = static_cast<DataBaseType &>(jdata_);
    
    CV q = LieGroupType().random();
    jmodel.calc(jdata.derived(),q);
    SE3 M_ref(jdata.M());
    
    CV q_int(q);
    const Eigen::DenseIndex nv = jdata.S().nv();
    TV v(nv); v.setZero();
    double eps = 1e-8;
    
    Eigen::Matrix<double,6,JointModel::NV> S(6,nv), S_ref(jdata.S().matrix());
    
    for(int k=0;k<nv;++k)
    {
      v[k] = eps;
      q_int = LieGroupType().integrate(q,v);
      jmodel.calc(jdata.derived(),q_int);
      SE3 M_int = jdata.M();
      
      S.col(k) = log6(M_ref.inverse()*M_int).toVector();
      S.col(k) /= eps;
      
      v[k] = 0.;
    }
    
    BOOST_CHECK(S.isApprox(S_ref,eps*1e1));
    std::cout << "S_ref:\n" << S_ref << std::endl;
    std::cout << "S:\n" << S << std::endl;
  }
};

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE (test_S_finit_diff)
{
  boost::mpl::for_each<JointModelVariant::types>(FiniteDiffJoint());
}

BOOST_AUTO_TEST_CASE (test_jacobian_vs_finit_diff)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);
  
  VectorXd q = VectorXd::Ones(model.nq);
  q.segment<4>(3).normalize();
  computeJointJacobians(model,data,q);

  Model::Index idx = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1);
  Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);
  
  getJointJacobian(model,data,idx,WORLD,Jrh);
  Data::Matrix6x Jrh_finite_diff = finiteDiffJacobian<false>(model,data,q,idx);
  BOOST_CHECK(Jrh_finite_diff.isApprox(Jrh,1e-8*1e1));
  
  getJointJacobian(model,data,idx,LOCAL,Jrh);
  Jrh_finite_diff = finiteDiffJacobian<true>(model,data,q,idx);
  BOOST_CHECK(Jrh_finite_diff.isApprox(Jrh,1e-8*1e1));
}

BOOST_AUTO_TEST_SUITE_END()
