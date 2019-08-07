//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/autodiff/cppad.hpp"

#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-algo.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_jointRX_motion_space)
{
  using CppAD::AD;
  using CppAD::NearEqual;

  typedef AD<double> AD_double;
  typedef pinocchio::JointCollectionDefaultTpl<AD_double> JointCollectionAD;
  typedef pinocchio::JointCollectionDefaultTpl<double> JointCollection;
  
  typedef pinocchio::SE3Tpl<AD_double> SE3AD;
  typedef pinocchio::MotionTpl<AD_double> MotionAD;
  typedef pinocchio::SE3Tpl<double> SE3;
  typedef pinocchio::MotionTpl<double> Motion;
  typedef pinocchio::ConstraintTpl<Eigen::Dynamic,double> ConstraintXd;
  
  typedef Eigen::Matrix<AD_double,Eigen::Dynamic,1> VectorXAD;
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
  
  typedef JointCollectionAD::JointModelRX JointModelRXAD;
  typedef JointModelRXAD::ConfigVector_t ConfigVectorAD;
//  typedef JointModelRXAD::TangentVector_t TangentVectorAD;
  typedef JointCollectionAD::JointDataRX JointDataRXAD;
  
  typedef JointCollection::JointModelRX JointModelRX;
  typedef JointModelRX::ConfigVector_t ConfigVector;
  typedef JointModelRX::TangentVector_t TangentVector;
  typedef JointCollection::JointDataRX JointDataRX;
  
  JointModelRX jmodel; jmodel.setIndexes(0,0,0);
  JointDataRX jdata(jmodel.createData());
  
  JointModelRXAD jmodel_ad = jmodel.cast<AD_double>();
  JointDataRXAD jdata_ad(jmodel_ad.createData());
  
  typedef pinocchio::LieGroup<JointModelRX>::type JointOperation;
  ConfigVector q(jmodel.nq()); JointOperation().random(q);
  ConfigVectorAD q_ad(q.cast<AD_double>());
  
   // Zero order
  jmodel_ad.calc(jdata_ad,q_ad);
  jmodel.calc(jdata,q);
  
  SE3 M1(jdata.M);
  SE3AD M2(jdata_ad.M);
  BOOST_CHECK(M1.isApprox(M2.cast<double>()));
  
  // First order
  TangentVector v(TangentVector::Random(jmodel.nv()));
  VectorXAD X(jmodel_ad.nv());
  
  for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
  {
    X[k] = v[k];
  }
  CppAD::Independent(X);
  jmodel_ad.calc(jdata_ad,q_ad,X);
  jmodel.calc(jdata,q,v);
  VectorXAD Y(6);
  MotionAD m_ad(jdata_ad.v);
  Motion m(jdata.v);
  ConstraintXd Sref(jdata.S.matrix());
  
  for(Eigen::DenseIndex k = 0; k < 3; ++k)
  {
    Y[k+Motion::LINEAR] = m_ad.linear()[k];
    Y[k+Motion::ANGULAR] = m_ad.angular()[k];
  }

  CppAD::ADFun<double> vjoint(X,Y);
  
  CPPAD_TESTVECTOR(double) x((size_t)jmodel_ad.nv());
  for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
  {
    x[(size_t)k] = v[k];
  }
  
  CPPAD_TESTVECTOR(double) jac = vjoint.Jacobian(x);
  MatrixX S(6,jac.size()/6);
  S = Eigen::Map<MatrixX>(jac.data(),S.rows(),S.cols());
  
  BOOST_CHECK(m.isApprox(m_ad.cast<double>()));
  
  BOOST_CHECK(Sref.matrix().isApprox(S));
}

struct TestADOnJoints
{
  template<typename JointModel>
  void operator()(const pinocchio::JointModelBase<JointModel> &) const
  {
    JointModel jmodel;
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  template<typename Scalar, int Options>
  void operator()(const pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> & ) const
  {
    typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar,Options> JointModel;
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  template<typename Scalar, int Options>
  void operator()(const pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> & ) const
  {
    typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar,Options> JointModel;
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  template<typename Scalar, int Options>
  void operator()(const pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> & ) const
  {
    typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar,Options> JointModel;
    typedef typename JointModel::Vector3 Vector3;
    JointModel jmodel(Vector3::Random().normalized());
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollection>
  void operator()(const pinocchio::JointModelTpl<Scalar,Options,JointCollection> & ) const
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef pinocchio::JointModelTpl<Scalar,Options,JointCollection> JointModel;
    JointModel jmodel((JointModelRX()));
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
 
  // TODO: implement it
  template<typename JointModel_>
  void operator()(const pinocchio::JointModelMimic<JointModel_> & /*jmodel*/) const
  {
    /* do nothing */
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollection>
  void operator()(const pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> & ) const
  {
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
    typedef pinocchio::JointModelCompositeTpl<Scalar,Options,JointCollection> JointModel;
    JointModel jmodel((JointModelRX()));
    jmodel.addJoint(JointModelRY());
    jmodel.setIndexes(0,0,0);
    
    test(jmodel);
  }
  
  template<typename JointModel>
  static void test(const pinocchio::JointModelBase<JointModel> & jmodel)
  {
    using CppAD::AD;
    using CppAD::NearEqual;
    
    typedef typename JointModel::Scalar Scalar;
    typedef typename JointModel::JointDataDerived JointData;
    
    typedef AD<Scalar> AD_scalar;

    typedef pinocchio::SE3Tpl<AD_scalar> SE3AD;
    typedef pinocchio::MotionTpl<AD_scalar> MotionAD;
    typedef pinocchio::SE3Tpl<Scalar> SE3;
    typedef pinocchio::MotionTpl<Scalar> Motion;
    typedef pinocchio::ConstraintTpl<Eigen::Dynamic,Scalar> ConstraintXd;
    
    typedef Eigen::Matrix<AD_scalar,Eigen::Dynamic,1> VectorXAD;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
    
    typedef typename pinocchio::CastType<AD_scalar,JointModel>::type JointModelAD;
    typedef typename JointModelAD::JointDataDerived JointDataAD;
    
    typedef typename JointModelAD::ConfigVector_t ConfigVectorAD;

    typedef typename JointModel::ConfigVector_t ConfigVector;
    typedef typename JointModel::TangentVector_t TangentVector;
    
    JointData jdata(jmodel.createData());
    pinocchio::JointDataBase<JointData> & jdata_base = jdata;
    
    JointModelAD jmodel_ad = jmodel.template cast<AD_scalar>();
    JointDataAD jdata_ad(jmodel_ad.createData());
    pinocchio::JointDataBase<JointDataAD> & jdata_ad_base = jdata_ad;
    
    ConfigVector q(jmodel.nq());
    ConfigVector lb(ConfigVector::Constant(jmodel.nq(),-1.));
    ConfigVector ub(ConfigVector::Constant(jmodel.nq(),1.));
    
    typedef pinocchio::RandomConfigurationStep<pinocchio::LieGroupMap,ConfigVector,ConfigVector,ConfigVector> RandomConfigAlgo;
    RandomConfigAlgo::run(jmodel.derived(),typename RandomConfigAlgo::ArgsType(q,lb,ub));
    
    ConfigVectorAD q_ad(q.template cast<AD_scalar>());
    
    // Zero order
    jmodel_ad.calc(jdata_ad,q_ad);
    jmodel.calc(jdata,q);
    
    SE3 M1(jdata_base.M());
    SE3AD M2(jdata_ad_base.M());
    BOOST_CHECK(M1.isApprox(M2.template cast<Scalar>()));
    
    // First order
    TangentVector v(TangentVector::Random(jmodel.nv()));
    VectorXAD X(jmodel_ad.nv());

    for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
    {
      X[k] = v[k];
    }
    CppAD::Independent(X);
    jmodel_ad.calc(jdata_ad,q_ad,X);
    jmodel.calc(jdata,q,v);
    VectorXAD Y(6);
    MotionAD m_ad(jdata_ad_base.v());
    Motion m(jdata_base.v());
    ConstraintXd Sref(jdata_base.S().matrix());

    for(Eigen::DenseIndex k = 0; k < 3; ++k)
    {
      Y[k+Motion::LINEAR] = m_ad.linear()[k];
      Y[k+Motion::ANGULAR] = m_ad.angular()[k];
    }

    CppAD::ADFun<Scalar> vjoint(X,Y);

    CPPAD_TESTVECTOR(Scalar) x((size_t)jmodel_ad.nv());
    for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
    {
      x[(size_t)k] = v[k];
    }

    CPPAD_TESTVECTOR(Scalar) jac = vjoint.Jacobian(x);
    MatrixX S(6,jac.size()/6);
    S = Eigen::Map<typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixX)>(jac.data(),S.rows(),S.cols());

    BOOST_CHECK(m.isApprox(m_ad.template cast<Scalar>()));

    BOOST_CHECK(Sref.matrix().isApprox(S));
  }
};

BOOST_AUTO_TEST_CASE(test_all_joints)
{
  typedef pinocchio::JointCollectionDefault::JointModelVariant JointModelVariant;
  boost::mpl::for_each<JointModelVariant::types>(TestADOnJoints());

  TestADOnJoints()(pinocchio::JointModel());
}

BOOST_AUTO_TEST_SUITE_END()
