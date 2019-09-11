//
// Copyright (c) 2015-2019 CNRS INRIA
//

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/force-set.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "utils/macros.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE (test_ForceSet)
{
  using namespace pinocchio;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  ForceSet F(12);
  ForceSet F2(Eigen::Matrix<double,3,2>::Zero(),Eigen::Matrix<double,3,2>::Zero());
  F.block(10,2) = F2;
  BOOST_CHECK_EQUAL(F.matrix().col(10).norm() , 0.0 );
  BOOST_CHECK(std::isnan(F.matrix()(0,9)));

  ForceSet F3(Eigen::Matrix<double,3,12>::Random(),Eigen::Matrix<double,3,12>::Random());
  ForceSet F4 = amb.act(F3);
  SE3::Matrix6 aXb= amb;
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix()).isApprox(F4.matrix(), 1e-12));


  ForceSet bF = bmc.act(F3);
  ForceSet aF = amb.act(bF); 
  ForceSet aF2 = amc.act(F3);
  BOOST_CHECK(aF.matrix().isApprox(aF2.matrix(), 1e-12));

  ForceSet F36 = amb.act(F3.block(3,6));
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)).isApprox(F36.matrix(), 1e-12));

  
  ForceSet F36full(12); F36full.block(3,6) = amb.act(F3.block(3,6)); 
  BOOST_CHECK((aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)).isApprox(F36full.matrix().block(0,3,6,6),
                                                            1e-12));
}

BOOST_AUTO_TEST_CASE ( test_ConstraintRX )
{
  using namespace pinocchio;

  Inertia Y = Inertia::Random();
  JointDataRX::Constraint_t S;

  ForceSet F1(1); F1.block(0,1) = Y*S;
  BOOST_CHECK(F1.matrix().isApprox(Y.matrix().col(3), 1e-12));

  ForceSet F2( Eigen::Matrix<double,3,9>::Random(),Eigen::Matrix<double,3,9>::Random() );
  Eigen::MatrixXd StF2 = S.transpose()*F2.block(5,3).matrix();
  BOOST_CHECK(StF2.isApprox(S.matrix().transpose()*F2.matrix().block(0,5,6,3)
                            , 1e-12));
}

template<typename JointModel>
void test_jmodel_nq_against_nq_ref(const JointModelBase<JointModel> & jmodel,
                                   const int & nq_ref)
{
  BOOST_CHECK(jmodel.nq() == nq_ref);
}

template<typename JointModel>
void test_jmodel_nq_against_nq_ref(const JointModelMimic<JointModel> & jmodel,
                                   const int & nq_ref)
{
  BOOST_CHECK(jmodel.jmodel().nq() == nq_ref);
}

template<typename JointModel, typename ConstraintDerived>
void test_nv_against_jmodel(const JointModelBase<JointModel> & jmodel,
                            const ConstraintBase<ConstraintDerived> & constraint)
{
  BOOST_CHECK(constraint.nv() == jmodel.nv());
}

template<typename JointModel, typename ConstraintDerived>
void test_nv_against_jmodel(const JointModelMimic<JointModel> & jmodel,
                            const ConstraintBase<ConstraintDerived> & constraint)
{
  BOOST_CHECK(constraint.nv() == jmodel.jmodel().nv());
}

template<class JointModel>
struct buildModel
{
  static Model run(const JointModelBase<JointModel> & jmodel)
  {
    Model model;
    model.addJoint(0,jmodel,SE3::Identity(),"joint");
    
    return model;
  }
};

template<class JointModel>
struct buildModel< JointModelMimic<JointModel> >
{
  typedef JointModelMimic<JointModel> JointModel_;
  
  static Model run(const JointModel_ & jmodel)
  {
    Model model;
    model.addJoint(0,jmodel.jmodel(),SE3::Identity(),"joint");
    model.addJoint(0,jmodel,SE3::Identity(),"joint_mimic");
    
    return model;
  }
};

template<typename JointModel>
void test_constraint_operations(const JointModelBase<JointModel> & jmodel)
{
  typedef typename traits<JointModel>::JointDerived Joint;
  typedef typename traits<Joint>::Constraint_t ConstraintType;
  typedef typename traits<Joint>::JointDataDerived JointData;
  typedef Eigen::Matrix<typename JointModel::Scalar,6,Eigen::Dynamic> Matrix6x;
  
  JointData jdata = jmodel.createData();
  typedef typename JointModel::ConfigVector_t ConfigVector_t;
  ConfigVector_t q;
  
  // We need to use a model here in order to call the randomConfiguration to init q.
  Model model = buildModel<JointModel>::run(jmodel.derived());
  
  test_jmodel_nq_against_nq_ref(jmodel.derived(),model.nq);
  
  q = randomConfiguration(model,
                          ConfigVector_t::Constant(model.nq,-1.),
                          ConfigVector_t::Constant(model.nq, 1.));
  
  // By calling jmodel.calc, we then have jdata.S which is initialized with non NaN quantities
  jmodel.calc(jdata,q);

  ConstraintType constraint(jdata.S);
  
  test_nv_against_jmodel(jmodel.derived(),constraint);
  BOOST_CHECK(constraint.cols() == constraint.nv());
  BOOST_CHECK(constraint.rows() == 6);
  
  typedef typename JointModel::TangentVector_t TangentVector_t;
  TangentVector_t v = TangentVector_t::Random(constraint.nv());

  typename ConstraintType::DenseBase constraint_mat = constraint.matrix();
  Motion m = constraint * v;
  Motion m_ref = Motion(constraint_mat * v);
  
  BOOST_CHECK(m.isApprox(m_ref));

  // Test SE3 action
  {
    SE3 M = SE3::Random();
    typename ConstraintType::DenseBase S = M.act(constraint);
    typename ConstraintType::DenseBase S_ref(6,constraint.nv());
    
    for(Eigen::DenseIndex k = 0; k < constraint.nv(); ++k)
    {
      typedef typename ConstraintType::DenseBase::ColXpr Vector6Like;
      MotionRef<Vector6Like> m_in(constraint_mat.col(k)), m_out(S_ref.col(k));
      
      m_out = M.act(m_in);
    }

    BOOST_CHECK(S.isApprox(S_ref));
  }
  
  // Test SE3 action inverse
  {
    SE3 M = SE3::Random();
    typename ConstraintType::DenseBase S = M.actInv(constraint);
    typename ConstraintType::DenseBase S_ref(6,constraint.nv());
    
    for(Eigen::DenseIndex k = 0; k < constraint.nv(); ++k)
    {
      typedef typename ConstraintType::DenseBase::ColXpr Vector6Like;
      MotionRef<Vector6Like> m_in(constraint_mat.col(k)), m_out(S_ref.col(k));
      
      m_out = M.actInv(m_in);
    }
    
    BOOST_CHECK(S.isApprox(S_ref));
  }
  
  // Test SE3 action and SE3 action inverse
  {
    const SE3 M = SE3::Random();
    const SE3 Minv = M.inverse();
    
    typename ConstraintType::DenseBase S1_vice = M.actInv(constraint);
    typename ConstraintType::DenseBase S2_vice = Minv.act(constraint);
    
    BOOST_CHECK(S1_vice.isApprox(S2_vice));
    
    typename ConstraintType::DenseBase S1_versa = M.act(constraint);
    typename ConstraintType::DenseBase S2_versa = Minv.actInv(constraint);
    
    BOOST_CHECK(S1_versa.isApprox(S2_versa));
    
  }
  
  // Test Motion action
  {
    Motion v = Motion::Random();
    
    typename ConstraintType::DenseBase S = v.cross(constraint);
    typename ConstraintType::DenseBase S_ref(6,constraint.nv());
    
    for(Eigen::DenseIndex k = 0; k < constraint.nv(); ++k)
    {
      typedef typename ConstraintType::DenseBase::ColXpr Vector6Like;
      MotionRef<Vector6Like> m_in(constraint_mat.col(k)), m_out(S_ref.col(k));
      
      m_out = v.cross(m_in);
    }
    BOOST_CHECK(S.isApprox(S_ref));
  }

  // Test transpose operations
  {
    const Eigen::DenseIndex dim = 20;
    const Matrix6x Fin = Matrix6x::Random(6,dim);
    Eigen::MatrixXd Fout = constraint.transpose() * Fin;
    Eigen::MatrixXd Fout_ref = constraint_mat.transpose() * Fin;
    BOOST_CHECK(Fout.isApprox(Fout_ref));

    Force force_in(Force::Random());
    Eigen::MatrixXd Stf = (constraint.transpose() * force_in);
    Eigen::MatrixXd Stf_ref = constraint_mat.transpose() * force_in.toVector();
    BOOST_CHECK(Stf_ref.isApprox(Stf));
  }
  
  // CRBA operations
  {
    const Inertia Y = Inertia::Random();
    Eigen::MatrixXd YS = Y * constraint;
    Eigen::MatrixXd YS_ref = Y.matrix() * constraint_mat;
    BOOST_CHECK(YS.isApprox(YS_ref));
  }
  
  // ABA operations
  {
    const Inertia Y = Inertia::Random();
    const Inertia::Matrix6 Y_mat = Y.matrix();
    Eigen::MatrixXd YS = Y_mat * constraint;
    Eigen::MatrixXd YS_ref = Y_mat * constraint_mat;
    BOOST_CHECK(YS.isApprox(YS_ref));
  }
  
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
    typedef pinocchio::JointModelRevoluteTpl<Scalar,Options,2> JointModelRZ;
    
    JointModel jmodel(JointModelRX(),SE3::Random());
    jmodel.addJoint(JointModelRY(),SE3::Random());
    jmodel.addJoint(JointModelRZ(),SE3::Random());
    
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

struct TestJointConstraint
{
  
  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> &) const
  {
    JointModel jmodel = init<JointModel>::run();
    jmodel.setIndexes(0,0,0);

    test_constraint_operations(jmodel);
  }
  
};

BOOST_AUTO_TEST_CASE(test_joint_constraint_operations)
{
  typedef JointCollectionDefault::JointModelVariant Variant;
  boost::mpl::for_each<Variant::types>(TestJointConstraint());
}

BOOST_AUTO_TEST_SUITE_END()
