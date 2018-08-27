// Copyright (c) 2017-2018, Joseph Mirabel, Justin Carpentier
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of pinocchio.
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio. If not, see <http://www.gnu.org/licenses/>.

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-variant-visitors.hpp"

#include "pinocchio/multibody/joint/joint.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/algorithm/string.hpp>

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, precision)                              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, precision),                            \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va).transpose() << "\n!=\n" << (Vb).transpose() << "\n]")

using namespace se3;

#define VERBOSE false
#define IFVERBOSE if(VERBOSE)

template <typename T>
void test_lie_group_methods (T & jmodel, typename T::JointDataDerived &)
{
  typedef double Scalar;
  
  const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision();
  IFVERBOSE std::cout << "Testing Joint over " << jmodel.shortname() << std::endl;
  typedef typename T::ConfigVector_t  ConfigVector_t;
  typedef typename T::TangentVector_t TangentVector_t;
  
  ConfigVector_t  q1(ConfigVector_t::Random (jmodel.nq()));
  TangentVector_t q1_dot(TangentVector_t::Random (jmodel.nv()));
  ConfigVector_t  q2(ConfigVector_t::Random (jmodel.nq()));
  
  typedef typename LieGroup<T>::type LieGroupType;
  static ConfigVector_t Ones(ConfigVector_t::Ones(jmodel.nq()));
  const Scalar u = 0.3;
  // se3::Inertia::Matrix6 Ia(se3::Inertia::Random().matrix());
  // bool update_I = false;
  
  q1 = LieGroupType().randomConfiguration(-Ones, Ones);
  
  
  typename T::JointDataDerived jdata = jmodel.createData();
  
  
  // Check integrate
  jmodel.calc(jdata, q1, q1_dot);
  SE3 M1 = jdata.M;
  Motion v1 = jdata.v;
  
  q2 = LieGroupType().integrate(q1,q1_dot);
  jmodel.calc(jdata,q2);
  SE3 M2 = jdata.M;
  
  SE3 M2_exp = M1*exp6(v1);
  
  if(jmodel.shortname() != "JointModelSphericalZYX")
  {
    BOOST_CHECK_MESSAGE(M2.isApprox(M2_exp), std::string("Error when integrating1 " + jmodel.shortname()));
  }
  
  // Check the reversability of integrate
  ConfigVector_t q3 = LieGroupType().integrate(q2,-q1_dot);
  jmodel.calc(jdata,q3);
  SE3 M3 = jdata.M;
  
  BOOST_CHECK_MESSAGE(M3.isApprox(M1), std::string("Error when integrating back " + jmodel.shortname()));
  
  // Check interpolate
  ConfigVector_t q_interpolate = LieGroupType().interpolate(q1,q2,0.);
  BOOST_CHECK_MESSAGE(q_interpolate.isApprox(q1), std::string("Error when interpolating " + jmodel.shortname()));
  
  q_interpolate = LieGroupType().interpolate(q1,q2,1.);
  BOOST_CHECK_MESSAGE(q_interpolate.isApprox(q2), std::string("Error when interpolating " + jmodel.shortname()));
  
  if(jmodel.shortname() != "JointModelSphericalZYX")
  {
    q_interpolate = LieGroupType().interpolate(q1,q2,u);
    jmodel.calc(jdata,q_interpolate);
    SE3 M_interpolate = jdata.M;
    
    SE3 M_interpolate_expected = M1*exp6(u*v1);
    BOOST_CHECK_MESSAGE(M_interpolate_expected.isApprox(M_interpolate,1e2*prec), std::string("Error when interpolating " + jmodel.shortname()));
  }

  // Check that difference between two equal configuration is exactly 0
  TangentVector_t zero = LieGroupType().difference(q1,q1);
  BOOST_CHECK_MESSAGE (zero.isZero (0), std::string ("Error: difference between two equal configurations is not 0."));
  zero = LieGroupType().difference(q2,q2);
  BOOST_CHECK_MESSAGE (zero.isZero (0), std::string ("Error: difference between two equal configurations is not 0."));

  // Check difference
  TangentVector_t vdiff = LieGroupType().difference(q1,q2);
  BOOST_CHECK_MESSAGE(vdiff.isApprox(q1_dot,1e2*prec), std::string("Error when differentiating " + jmodel.shortname()));
  
  // Check distance
  Scalar dist = LieGroupType().distance(q1,q2);
  BOOST_CHECK_MESSAGE(dist > 0., "distance - wrong results");
  BOOST_CHECK_SMALL(std::fabs(dist-q1_dot.norm()), 10*prec);
  
  std::string error_prefix("LieGroup");
  error_prefix += " on joint " + jmodel.shortname();
  
  BOOST_CHECK_MESSAGE(jmodel.nq() == LieGroupType::NQ, std::string(error_prefix + " - nq "));
  BOOST_CHECK_MESSAGE(jmodel.nv() == LieGroupType::NV, std::string(error_prefix + " - nv "));
  
  BOOST_CHECK_MESSAGE
  (jmodel.nq() ==
   LieGroupType().randomConfiguration(-1 * Ones, Ones).size(),
   std::string(error_prefix + " - RandomConfiguration dimensions "));

  ConfigVector_t q_normalize(ConfigVector_t::Random());
  Eigen::VectorXd q_normalize_ref(q_normalize);
  if(jmodel.shortname() == "JointModelSpherical")
  {
    q_normalize_ref /= q_normalize_ref.norm();
  }
  else if(jmodel.shortname() == "JointModelFreeFlyer")
  {
    q_normalize_ref.template tail<4>() /= q_normalize_ref.template tail<4>().norm();
  }
  else if(boost::algorithm::istarts_with(jmodel.shortname(),"JointModelRUB"))
  {
    q_normalize_ref /= q_normalize_ref.norm();
  }
  else if(jmodel.shortname() == "JointModelPlanar")
  {
    q_normalize_ref.template tail<2>() /= q_normalize_ref.template tail<2>().norm();
  }
  LieGroupType().normalize(q_normalize);
  BOOST_CHECK_MESSAGE(q_normalize.isApprox(q_normalize_ref), std::string(error_prefix + " - normalize "));
}

struct TestJoint{

  template <typename T>
  void operator()(const T ) const
  {
    T jmodel;
    jmodel.setIndexes(0,0,0);
    typename T::JointDataDerived jdata = jmodel.createData();

    test_lie_group_methods(jmodel, jdata);    
  }

  void operator()(const se3::JointModelRevoluteUnaligned & ) const
  {
    se3::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelRevoluteUnaligned::JointDataDerived jdata = jmodel.createData();

    test_lie_group_methods(jmodel, jdata);
  }

  void operator()(const se3::JointModelPrismaticUnaligned & ) const
  {
    se3::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelPrismaticUnaligned::JointDataDerived jdata = jmodel.createData();

    test_lie_group_methods(jmodel, jdata);
  }

};

struct LieGroup_Jdifference{
  template <typename T>
  void operator()(const T ) const
  {
    typedef typename T::ConfigVector_t ConfigVector_t;
    typedef typename T::TangentVector_t TangentVector_t;
    typedef typename T::JacobianMatrix_t JacobianMatrix_t;
    typedef typename T::Scalar Scalar;

    T lg;
    IFVERBOSE std::cout << lg.name() << std::endl;
    ConfigVector_t q[2], q_dv[2];
    q[0] = lg.random();
    q[1] = lg.random();
    TangentVector_t va, vb, dv;
    JacobianMatrix_t J[2];
    dv.setZero();

    lg.difference (q[0], q[1], va);
    lg.Jdifference (q[0], q[1], J[0], J[1]);

    const Scalar eps = 1e-6;
    for (int k = 0; k < 2; ++k) {
      IFVERBOSE std::cout << "Checking J" << k << '\n' << J[k] << std::endl;
      q_dv[0] = q[0];
      q_dv[1] = q[1];
      // Check J[k]
      for (int i = 0; i < dv.size(); ++i)
      {
        dv[i] = eps;
        lg.integrate (q[k], dv, q_dv[k]);
        lg.difference (q_dv[0], q_dv[1], vb);

        // vb - va ~ J[k] * dv

        TangentVector_t J_dv = J[k]*dv / eps;
        TangentVector_t vb_va = (vb - va) / eps;
        EIGEN_VECTOR_IS_APPROX (vb_va, J_dv, 1e-2);
        dv[i] = 0;
      }
    }
  }
};

struct LieGroup_Jintegrate{
  template <typename T>
  void operator()(const T ) const
  {
    typedef typename T::ConfigVector_t ConfigVector_t;
    typedef typename T::TangentVector_t TangentVector_t;
    typedef typename T::JacobianMatrix_t JacobianMatrix_t;
    typedef typename T::Scalar Scalar;

    T lg;
    ConfigVector_t q = lg.random();
    TangentVector_t v, dv;
    v.setRandom();
    dv.setZero();

    ConfigVector_t q_v = lg.integrate (q, v);

    JacobianMatrix_t Jq, Jv;
    lg.dIntegrate_dq (q, v, Jq);
    lg.dIntegrate_dv (q, v, Jv);

    const Scalar eps = 1e-6;
    for (int i = 0; i < v.size(); ++i)
    {
      dv[i] = eps;
      ConfigVector_t q_dv = lg.integrate (q, dv);

      ConfigVector_t q_dv_v = lg.integrate (q_dv, v);
      TangentVector_t Jq_dv = Jq*dv / eps;
      // q_dv_v - q_v ~ Jq dv
      TangentVector_t dI_dq = lg.difference (q_v, q_dv_v) / eps;
      EIGEN_VECTOR_IS_APPROX (dI_dq, Jq_dv, 1e-2);

      ConfigVector_t q_v_dv = lg.integrate (q, (v+dv).eval());
      TangentVector_t Jv_dv = Jv*dv / eps;
      // q_v_dv - q_v ~ Jv dv
      TangentVector_t dI_dv = lg.difference (q_v, q_v_dv) / eps;
      EIGEN_VECTOR_IS_APPROX (dI_dv, Jv_dv, 1e-2);

      dv[i] = 0;
    }
  }
};

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_all )
{
  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned
                          , JointModelSpherical, JointModelSphericalZYX
                          , JointModelPX, JointModelPY, JointModelPZ
                          , JointModelPrismaticUnaligned
                          , JointModelFreeFlyer
                          , JointModelPlanar
                          , JointModelTranslation
                          , JointModelRUBX, JointModelRUBY, JointModelRUBZ
                          > Variant;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Variant::types>(TestJoint());
  
  // FIXME JointModelComposite does not work.
  // boost::mpl::for_each<JointModelVariant::types>(TestJoint());
  
}

BOOST_AUTO_TEST_CASE ( Jdifference )
{
  typedef boost::mpl::vector<  VectorSpaceOperation<1>
                             , VectorSpaceOperation<2>
                             , SpecialOrthogonalOperation<2>
                             , SpecialOrthogonalOperation<3>
                             , SpecialEuclideanOperation<2>
                             , SpecialEuclideanOperation<3>
                             , CartesianProductOperation<
                                 VectorSpaceOperation<2>,
                                 SpecialOrthogonalOperation<2>
                               >
                             , CartesianProductOperation<
                                 VectorSpaceOperation<3>,
                                 SpecialOrthogonalOperation<3>
                               >
                             > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_Jdifference());
}

BOOST_AUTO_TEST_CASE ( Jintegrate )
{
  typedef boost::mpl::vector<  VectorSpaceOperation<1>
                             , VectorSpaceOperation<2>
                             , SpecialOrthogonalOperation<2>
                             , SpecialOrthogonalOperation<3>
                             , SpecialEuclideanOperation<2>
                             , SpecialEuclideanOperation<3>
                             , CartesianProductOperation<
                                 VectorSpaceOperation<2>,
                                 SpecialOrthogonalOperation<2>
                               >
                             , CartesianProductOperation<
                                 VectorSpaceOperation<3>,
                                 SpecialOrthogonalOperation<3>
                               >
                             > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_Jintegrate());
}

BOOST_AUTO_TEST_CASE ( test_vector_space )
{
  typedef VectorSpaceOperation<3> VSO_t;
  VSO_t::ConfigVector_t q,
    lo(VSO_t::ConfigVector_t::Constant(-std::numeric_limits<double>::infinity())),
    // lo(VSO_t::ConfigVector_t::Constant(                                       0)),
    // up(VSO_t::ConfigVector_t::Constant( std::numeric_limits<double>::infinity()));
    up(VSO_t::ConfigVector_t::Constant(                                       0));

  bool error = false;
  try {
    VSO_t ().randomConfiguration(lo, up, q);
  } catch (const std::runtime_error&) {
    error = true;
  }
  BOOST_CHECK_MESSAGE(error, "Random configuration between infinite bounds should return an error");
}

BOOST_AUTO_TEST_CASE ( test_size )
{
  // R^1: neutral = [0]
  VectorSpaceOperation <1> vs1;
  Eigen::VectorXd neutral;
  neutral.resize (1);
  neutral.setZero ();
  BOOST_CHECK (vs1.nq () == 1);
  BOOST_CHECK (vs1.nv () == 1);
  BOOST_CHECK (vs1.name () == "R^1");
  BOOST_CHECK (vs1.neutral () == neutral);
  // R^2: neutral = [0, 0]
  VectorSpaceOperation <2> vs2;
  neutral.resize (2);
  neutral.setZero ();
  BOOST_CHECK (vs2.nq () == 2);
  BOOST_CHECK (vs2.nv () == 2);
  BOOST_CHECK (vs2.name () == "R^2");
  BOOST_CHECK (vs2.neutral () == neutral);
  // R^3: neutral = [0, 0, 0]
  VectorSpaceOperation <3> vs3;
  neutral.resize (3);
  neutral.setZero ();
  BOOST_CHECK (vs3.nq () == 3);
  BOOST_CHECK (vs3.nv () == 3);
  BOOST_CHECK (vs3.name () == "R^3");
  BOOST_CHECK (vs3.neutral () == neutral);
  // SO(2): neutral = [1, 0]
  SpecialOrthogonalOperation <2> so2;
  neutral.resize (2); neutral [0] = 1; neutral [1] = 0;
  BOOST_CHECK (so2.nq () == 2);
  BOOST_CHECK (so2.nv () == 1);
  BOOST_CHECK (so2.name () == "SO(2)");
  BOOST_CHECK (so2.neutral () == neutral);
  // SO(3): neutral = [0, 0, 0, 1]
  SpecialOrthogonalOperation <3> so3;
  neutral.resize (4); neutral.setZero ();
  neutral [3] = 1;
  BOOST_CHECK (so3.nq () == 4);
  BOOST_CHECK (so3.nv () == 3);
  BOOST_CHECK (so3.name () == "SO(3)");
  BOOST_CHECK (so3.neutral () == neutral);
  // SE(2): neutral = [0, 0, 1, 0]
  SpecialEuclideanOperation <2> se2;
  neutral.resize (4); neutral.setZero ();
  neutral [2] = 1;
  BOOST_CHECK (se2.nq () == 4);
  BOOST_CHECK (se2.nv () == 3);
  BOOST_CHECK (se2.name () == "SE(2)");
  BOOST_CHECK (se2.neutral () == neutral);
  // SE(3): neutral = [0, 0, 0, 0, 0, 0, 1]
  SpecialEuclideanOperation <3> se3;
  neutral.resize (7); neutral.setZero ();
  neutral [6] = 1;
  BOOST_CHECK (se3.nq () == 7);
  BOOST_CHECK (se3.nv () == 6);
  BOOST_CHECK (se3.name () == "SE(3)");
  BOOST_CHECK (se3.neutral () == neutral);
  // R^2 x SO(2): neutral = [0, 0, 1, 0]
  CartesianProductOperation <VectorSpaceOperation <2>,
                             SpecialOrthogonalOperation <2> > r2xso2;
  neutral.resize (4); neutral.setZero ();
  neutral [2] = 1;
  BOOST_CHECK (r2xso2.nq () == 4);
  BOOST_CHECK (r2xso2.nv () == 3);
  BOOST_CHECK (r2xso2.name () == "R^2*SO(2)");
  BOOST_CHECK (r2xso2.neutral () == neutral);
  // R^3 x SO(3): neutral = [0, 0, 0, 0, 0, 0, 1]
  CartesianProductOperation <VectorSpaceOperation <3>,
                             SpecialOrthogonalOperation <3> > r3xso3;
  neutral.resize (7); neutral.setZero ();
  neutral [6] = 1;
  BOOST_CHECK (r3xso3.nq () == 7);
  BOOST_CHECK (r3xso3.nv () == 6);
  BOOST_CHECK (r3xso3.name () == "R^3*SO(3)");
  BOOST_CHECK (r3xso3.neutral () == neutral);
}

BOOST_AUTO_TEST_CASE(test_dim_computation)
{
  int dim = eval_set_dim<1,1>::value ;
  BOOST_CHECK(dim == 2);
  dim = eval_set_dim<Eigen::Dynamic,1>::value;
  BOOST_CHECK(dim == Eigen::Dynamic);
  dim = eval_set_dim<1,Eigen::Dynamic>::value;
  BOOST_CHECK(dim == Eigen::Dynamic);
}

struct TestLieGroupVariantVisitor
{
  template<typename Derived>
  void operator() (const LieGroupBase<Derived> & lg) const
  {
    LieGroupVariant lg_variant(lg.derived());
    test(lg,lg_variant);
  }
  
  template<typename Derived>
  static void test(const LieGroupBase<Derived> & lg, const LieGroupVariant & lg_variant)
  {
    typedef typename Derived::ConfigVector_t ConfigVector_t;
    typedef typename Derived::TangentVector_t TangentVector_t;
    
    BOOST_CHECK(lg.nq() == nq(lg_variant));
    BOOST_CHECK(lg.nv() == nv(lg_variant));
    
    BOOST_CHECK(lg.name() == name(lg_variant));
    
    BOOST_CHECK(lg.neutral() == neutral(lg_variant));
    
    ConfigVector_t q0 = lg.random();
    TangentVector_t v = TangentVector_t::Random(lg.nv());
    ConfigVector_t qout_ref(lg.nq());
    lg.integrate(q0, v, qout_ref);
    
    typedef Eigen::VectorXd ConfigVectorGeneric;
    typedef Eigen::VectorXd TangentVectorGeneric;
    ConfigVectorGeneric qout(lg.nq());
    integrate(lg_variant, ConfigVectorGeneric(q0), TangentVectorGeneric(v), qout);
    BOOST_CHECK(qout.isApprox(qout_ref));
  }
};

BOOST_AUTO_TEST_CASE(test_liegroup_variant)
{
  boost::mpl::for_each<LieGroupVariant::types>(TestLieGroupVariantVisitor());
}

BOOST_AUTO_TEST_SUITE_END ()
