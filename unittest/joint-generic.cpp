//
// Copyright (c) 2016-2019 CNRS INRIA
//

#include "pinocchio/multibody/joint/joint-generic.hpp"

#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include "pinocchio/multibody/model.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

template <typename JointModel>
void test_joint_methods(JointModelBase<JointModel> & jmodel,
                        JointDataBase<typename JointModel::JointDataDerived> & jdata)
{
  typedef typename LieGroup<JointModel>::type LieGroupType;

  std::cout << "Testing Joint over " << jmodel.shortname() << std::endl;
  Eigen::VectorXd q1, q2;
  Eigen::VectorXd v1(Eigen::VectorXd::Random(jdata.S().nv()));
  Inertia::Matrix6 Ia(pinocchio::Inertia::Random().matrix());
  bool update_I = false;

  q1 = LieGroupType().random();
  q2 = LieGroupType().random();

  jmodel.calc(jdata.derived(), q1, v1);
  jmodel.calc_aba(jdata.derived(), Ia, update_I);

  pinocchio::JointModel jma(jmodel);
  pinocchio::JointData jda(jdata.derived());

  jma.calc(jda, q1, v1);
  jma.calc_aba(jda, Ia, update_I);

  std::string error_prefix("JointModel on " + jma.shortname());
  BOOST_CHECK_MESSAGE(jmodel.nq() == jma.nq() ,std::string(error_prefix + " - nq "));
  BOOST_CHECK_MESSAGE(jmodel.nv() == jma.nv() ,std::string(error_prefix + " - nv "));

  BOOST_CHECK_MESSAGE(jmodel.idx_q() == jma.idx_q() ,std::string(error_prefix + " - Idx_q "));
  BOOST_CHECK_MESSAGE(jmodel.idx_v() == jma.idx_v() ,std::string(error_prefix + " - Idx_v "));
  BOOST_CHECK_MESSAGE(jmodel.id() == jma.id() ,std::string(error_prefix + " - JointId "));

  BOOST_CHECK_MESSAGE(jda.S().matrix().isApprox(jdata.S().matrix()),std::string(error_prefix + " - ConstraintXd "));
  BOOST_CHECK_MESSAGE( (jda.M()).isApprox((jdata.M())),std::string(error_prefix + " - Joint transforms ")); // ==  or isApprox ?
  BOOST_CHECK_MESSAGE( (jda.v()).isApprox( (pinocchio::Motion(jdata.v()))),std::string(error_prefix + " - Joint motions "));
  BOOST_CHECK_MESSAGE((jda.c()) == (jdata.c()),std::string(error_prefix + " - Joint bias "));

  BOOST_CHECK_MESSAGE((jda.U()).isApprox(jdata.U()),std::string(error_prefix + " - Joint U inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jda.Dinv()).isApprox(jdata.Dinv()),std::string(error_prefix + " - Joint DInv inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jda.UDinv()).isApprox(jdata.UDinv()),std::string(error_prefix + " - Joint UDInv inertia matrix decomposition "));

  // Test vxS
  typedef typename JointModel::Constraint_t Constraint_t;
  typedef typename Constraint_t::DenseBase ConstraintDense;

  Motion v(Motion::Random());
  ConstraintDense vxS(v.cross(jdata.S()));
  ConstraintDense vxS_ref = v.toActionMatrix() * jdata.S().matrix();

  BOOST_CHECK_MESSAGE(vxS.isApprox(vxS_ref),std::string(error_prefix + "- Joint vxS operation "));

  // Test Y*S
  const Inertia Isparse(Inertia::Random());
  const Inertia::Matrix6 Idense(Isparse.matrix());

  const ConstraintDense IsparseS = Isparse * jdata.S();
  const ConstraintDense IdenseS = Idense * jdata.S();

  BOOST_CHECK_MESSAGE(IdenseS.isApprox(IsparseS),std::string(error_prefix + "- Joint YS operation "));

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

struct TestJoint{

  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> & ) const
  {
    JointModel jmodel = init<JointModel>::run();
    jmodel.setIndexes(0,0,0);
    typename JointModel::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }
  
  void operator()(const pinocchio::JointModelComposite & ) const
  {
    
  }

};

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl> struct JointTest;
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl> struct JointModelTest;
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl> struct JointDataTest;
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct traits< JointDataTest<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct traits< JointModelTest<Scalar,Options,JointCollectionTpl> >
  { typedef JointTpl<Scalar,Options,JointCollectionTpl> JointDerived; };
  
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  struct traits< JointTest<_Scalar,_Options,JointCollectionTpl> >
  {
    enum {
      Options = _Options,
      NQ = Eigen::Dynamic, // Dynamic because unknown at compile time
      NV = Eigen::Dynamic
    };
    typedef _Scalar Scalar;
    
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointDataDerived;
    typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModelDerived;
    typedef ConstraintTpl<Eigen::Dynamic,Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTpl<Scalar,Options>  Motion_t;
    typedef MotionTpl<Scalar,Options>  Bias_t;
    
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> U_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> UD_t;
    
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> TangentVector_t;
  };
  
  template<typename _Scalar, int _Options, template<typename,int> class JointCollectionTpl>
  struct JointModelTest
  : JointCollectionTpl<_Scalar,_Options>::JointModelVariant
  , JointModelBase< JointModelTest<_Scalar,_Options,JointCollectionTpl> >
  {
    typedef JointTest<_Scalar,_Options,JointCollectionTpl> JointDerived;
    typedef JointCollectionTpl<_Scalar,_Options> JointCollection;
    typedef typename JointCollection::JointModelVariant VariantBase;
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    
    PINOCCHIO_JOINT_TYPEDEF_TEMPLATE(JointDerived);
    
    JointModelTest(const JointModelVariant & jmodel)
    : VariantBase(jmodel)
    {}
    
  };

}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(test_joint_from_joint_composite)
{
  typedef JointCollectionDefault JointCollection;
  typedef JointCollection::JointModelVariant JointModelVariant;
  
  JointModelRX jmodel_revolute_x;
  JointModel jmodel_generic(jmodel_revolute_x);
  JointModelVariant jmodel_variant(jmodel_revolute_x);
  
  JointModelTest<double,0,JointCollectionDefaultTpl> jmodel_test(jmodel_revolute_x);
  std::vector< JointModelTest<double,0,JointCollectionDefaultTpl> > jmodel_test_vector;
  jmodel_test_vector.push_back(JointModelTest<double,0,JointCollectionDefaultTpl>(jmodel_revolute_x));
  
  std::vector<JointModelVariant> jmodel_variant_vector;
  jmodel_variant_vector.push_back(jmodel_revolute_x);
  
  std::vector<JointModel> jmodel_generic_vector;
  jmodel_generic_vector.push_back((JointModel)jmodel_revolute_x);
  JointModelComposite jmodel_composite(jmodel_revolute_x);
}

BOOST_AUTO_TEST_CASE ( test_all_joints )
{
  boost::mpl::for_each<JointModelVariant::types>(TestJoint());
}

BOOST_AUTO_TEST_CASE(test_empty_model)
{
  JointModel jmodel;
  std::cout << "nq " << jmodel.nq() << std::endl;
  std::cout << "nv " << jmodel.nv() << std::endl;
  std::cout << "idx_q " << jmodel.idx_q() << std::endl;
  std::cout << "idx_v " << jmodel.idx_v() << std::endl;
  std::cout << "id " << jmodel.id() << std::endl;
  std::cout << "name " << jmodel.shortname() << std::endl;

  BOOST_CHECK(jmodel.idx_q() == -1);
  BOOST_CHECK(jmodel.idx_v() == -1);
}

BOOST_AUTO_TEST_CASE(isEqual)
{
  JointModelRX joint_revolutex;
  JointModelRY joint_revolutey;
  
  BOOST_CHECK(joint_revolutex != joint_revolutey);
  
  JointModel jmodelx(joint_revolutex);
  jmodelx.setIndexes(0,0,0);
  
  JointModel jmodelx_copy = jmodelx;
  BOOST_CHECK(jmodelx_copy == jmodelx.derived());
  
  JointModel jmodely(joint_revolutey);
  // TDDO: the comparison of two variants is not supported by some old version of BOOST
//  BOOST_CHECK(jmodely.toVariant() != jmodelx.toVariant());
  BOOST_CHECK(jmodely != jmodelx);
}

BOOST_AUTO_TEST_CASE(cast)
{
  JointModelRX joint_revolutex;
  
  JointModel jmodelx(joint_revolutex);
  jmodelx.setIndexes(0,0,0);
  
  BOOST_CHECK(jmodelx.cast<double>() == jmodelx);
  BOOST_CHECK(jmodelx.cast<long double>().cast<double>() == jmodelx);
}

struct TestJointOperatorEqual
{

  template <typename JointModel>
  void operator()(const JointModelBase<JointModel> & ) const
  {
    JointModel jmodel_init = init<JointModel>::run();
    typedef typename JointModel::JointDataDerived JointData;
    
    Model model;
    model.addJoint(0,jmodel_init,SE3::Random(),"toto");
    model.lowerPositionLimit.fill(-1.);
    model.upperPositionLimit.fill( 1.);
    
    const JointModel & jmodel = boost::get<JointModel>(model.joints[1]);
    
    Eigen::VectorXd q = randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    
    JointData jdata = jmodel.createData();
    
    jmodel.calc(jdata,q,v);
    Inertia::Matrix6 I = Inertia::Matrix6::Identity();
    jmodel.calc_aba(jdata,I,false);
    test(jdata);
  }
  
  template <typename JointModel>
  void operator()(const pinocchio::JointModelMimic<JointModel> & ) const
  {

  }
  
  template<typename JointData>
  static void test(const JointData & jdata)
  {
    pinocchio::JointData jdata_generic1(jdata);
    
    std::cout << "name: " << jdata_generic1.shortname() << std::endl;
    BOOST_CHECK(jdata_generic1 == jdata_generic1);
  }

};

BOOST_AUTO_TEST_CASE(test_operator_equal)
{
  boost::mpl::for_each<JointModelVariant::types>(TestJointOperatorEqual());
}

BOOST_AUTO_TEST_SUITE_END ()
