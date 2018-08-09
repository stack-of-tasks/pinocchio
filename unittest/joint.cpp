//
// Copyright (c) 2016-2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/multibody/joint/joint-generic.hpp"

#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;

template <typename JointModel>
void test_joint_methods(JointModel & jmodel, typename JointModel::JointDataDerived & jdata)
{
  typedef typename LieGroup<JointModel>::type LieGroupType;

  std::cout << "Testing Joint over " << jmodel.shortname() << std::endl;
  Eigen::VectorXd q1(Eigen::VectorXd::Random (jmodel.nq()));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random (jmodel.nv()));
  Eigen::VectorXd q2(Eigen::VectorXd::Random (jmodel.nq()));
  se3::Inertia::Matrix6 Ia(se3::Inertia::Random().matrix());
  bool update_I = false;

  q1 = LieGroupType().random();
  q2 = LieGroupType().random();

  jmodel.calc(jdata, q1, q1_dot);
  jmodel.calc_aba(jdata, Ia, update_I);

  se3::JointModel jma(jmodel);
  se3::JointData jda(jdata);

  jma.calc(jda, q1, q1_dot);
  jma.calc_aba(jda, Ia, update_I);

  std::string error_prefix("JointModel on " + jma.shortname());
  BOOST_CHECK_MESSAGE(jmodel.nq() == jma.nq() ,std::string(error_prefix + " - nq "));
  BOOST_CHECK_MESSAGE(jmodel.nv() == jma.nv() ,std::string(error_prefix + " - nv "));

  BOOST_CHECK_MESSAGE(jmodel.idx_q() == jma.idx_q() ,std::string(error_prefix + " - Idx_q "));
  BOOST_CHECK_MESSAGE(jmodel.idx_v() == jma.idx_v() ,std::string(error_prefix + " - Idx_v "));
  BOOST_CHECK_MESSAGE(jmodel.id() == jma.id() ,std::string(error_prefix + " - JointId "));

  BOOST_CHECK_MESSAGE(jda.S().matrix().isApprox(jdata.S.matrix()),std::string(error_prefix + " - ConstraintXd "));
  BOOST_CHECK_MESSAGE( (jda.M()).isApprox((jdata.M)),std::string(error_prefix + " - Joint transforms ")); // ==  or isApprox ?
  BOOST_CHECK_MESSAGE( (jda.v()).isApprox( (se3::Motion(jdata.v))),std::string(error_prefix + " - Joint motions "));
  BOOST_CHECK_MESSAGE((jda.c()) == (jdata.c),std::string(error_prefix + " - Joint bias "));

  BOOST_CHECK_MESSAGE((jda.U()).isApprox(jdata.U),std::string(error_prefix + " - Joint U inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jda.Dinv()).isApprox(jdata.Dinv),std::string(error_prefix + " - Joint DInv inertia matrix decomposition "));
  BOOST_CHECK_MESSAGE((jda.UDinv()).isApprox(jdata.UDinv),std::string(error_prefix + " - Joint UDInv inertia matrix decomposition "));

  // Test vxS
  typedef typename JointModel::Constraint_t Constraint_t;
  typedef typename Constraint_t::DenseBase ConstraintDense;

  Motion v(Motion::Random());
  ConstraintDense vxS(v.cross(jdata.S));
  ConstraintDense vxS_ref = v.toActionMatrix() * jdata.S.matrix();

  BOOST_CHECK_MESSAGE(vxS.isApprox(vxS_ref),std::string(error_prefix + "- Joint vxS operation "));

  // Test Y*S
  const Inertia Isparse(Inertia::Random());
  const Inertia::Matrix6 Idense(Isparse.matrix());

  const ConstraintDense IsparseS = Isparse * jdata.S;
  const ConstraintDense IdenseS = Idense * jdata.S;

  BOOST_CHECK_MESSAGE(IdenseS.isApprox(IsparseS),std::string(error_prefix + "- Joint YS operation "));

}

struct TestJoint{

  template <typename T>
  void operator()(const T ) const
  {
    T jmodel;
    jmodel.setIndexes(0,0,0);
    typename T::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }

  void operator()(const se3::JointModelComposite & ) const
  {
    // se3::JointModelComposite jmodel(2);
    // jmodel.addJointModel(se3::JointModelRX());
    // jmodel.addJointModel(se3::JointModelRY());

    se3::JointModelComposite jmodel((se3::JointModelRX()));
    jmodel.addJoint(se3::JointModelRY());
    jmodel.setIndexes(0,0,0);

    se3::JointModelComposite::JointDataDerived jdata = jmodel.createData();

    // TODO: fixme when LieGroups will be implemented for JointModelComposite
//    test_joint_methods(jmodel, jdata);
  }

  void operator()(const se3::JointModelRevoluteUnaligned & ) const
  {
    se3::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelRevoluteUnaligned::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }

  void operator()(const se3::JointModelPrismaticUnaligned & ) const
  {
    se3::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelPrismaticUnaligned::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }

};

namespace se3
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
    
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> F_t;
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
    
    SE3_JOINT_TYPEDEF_TEMPLATE;
    
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
  BOOST_CHECK(jmodely.toVariant() != jmodelx.toVariant());
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

BOOST_AUTO_TEST_SUITE_END ()
