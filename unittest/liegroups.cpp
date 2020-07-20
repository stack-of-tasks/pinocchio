// Copyright (c) 2017-2020, CNRS INRIA
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-collection.hpp"
#include "pinocchio/multibody/liegroup/liegroup-generic.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product-variant.hpp"

#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/algorithm/string.hpp>

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, precision)                              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, precision),                            \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va).transpose() << "\n!=\n" << (Vb).transpose() << "\n]")
#define EIGEN_MATRIX_IS_APPROX(Va, Vb, precision)                              \
  BOOST_CHECK_MESSAGE((Va).isApprox(Vb, precision),                            \
      "check " #Va ".isApprox(" #Vb ") failed "                                \
      "[\n" << (Va) << "\n!=\n" << (Vb) << "\n]")

using namespace pinocchio;

#define VERBOSE false
#define IFVERBOSE if(VERBOSE)

namespace pinocchio {
template<typename Derived>
std::ostream& operator<< (std::ostream& os, const LieGroupBase<Derived>& lg)
{
  return os << lg.name();
}
template<typename LieGroupCollection>
std::ostream& operator<< (std::ostream& os, const LieGroupGenericTpl<LieGroupCollection>& lg)
{
  return os << lg.name();
}
} // namespace pinocchio

template <typename T>
void test_lie_group_methods (T & jmodel, typename T::JointDataDerived &)
{
  typedef double Scalar;
  
  const Scalar prec = Eigen::NumTraits<Scalar>::dummy_precision();
  BOOST_TEST_MESSAGE ("Testing Joint over " << jmodel.shortname());
  typedef typename T::ConfigVector_t  ConfigVector_t;
  typedef typename T::TangentVector_t TangentVector_t;
  
  ConfigVector_t  q1(ConfigVector_t::Random (jmodel.nq()));
  TangentVector_t q1_dot(TangentVector_t::Random (jmodel.nv()));
  ConfigVector_t  q2(ConfigVector_t::Random (jmodel.nq()));
  
  typedef typename LieGroup<T>::type LieGroupType;
  static ConfigVector_t Ones(ConfigVector_t::Ones(jmodel.nq()));
  const Scalar u = 0.3;
  // pinocchio::Inertia::Matrix6 Ia(pinocchio::Inertia::Random().matrix());
  // bool update_I = false;
  
  q1 = LieGroupType().randomConfiguration(-Ones, Ones);
  
  typename T::JointDataDerived jdata = jmodel.createData();
  
  // Check integrate
  jmodel.calc(jdata, q1, q1_dot);
  SE3 M1 = jdata.M;
  Motion v1(jdata.v);
  
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
  BOOST_CHECK_MESSAGE (zero.isZero(), std::string ("Error: difference between two equal configurations is not 0."));
  zero = LieGroupType().difference(q2,q2);
  BOOST_CHECK_MESSAGE (zero.isZero(), std::string ("Error: difference between two equal configurations is not 0."));

  // Check difference
  TangentVector_t vdiff = LieGroupType().difference(q1,q2);
  BOOST_CHECK_MESSAGE(vdiff.isApprox(q1_dot,1e2*prec), std::string("Error when differentiating " + jmodel.shortname()));
  
  // Check distance
  Scalar dist = LieGroupType().distance(q1,q2);
  BOOST_CHECK_MESSAGE(dist > 0., "distance - wrong results");
  BOOST_CHECK_SMALL(math::fabs(dist-q1_dot.norm()), 10*prec);
  
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

  void operator()(const pinocchio::JointModelRevoluteUnaligned & ) const
  {
    pinocchio::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    pinocchio::JointModelRevoluteUnaligned::JointDataDerived jdata = jmodel.createData();

    test_lie_group_methods(jmodel, jdata);
  }

  void operator()(const pinocchio::JointModelPrismaticUnaligned & ) const
  {
    pinocchio::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    pinocchio::JointModelPrismaticUnaligned::JointDataDerived jdata = jmodel.createData();

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
    BOOST_TEST_MESSAGE (lg.name());
    ConfigVector_t q[2], q_dv[2];
    q[0] = lg.random();
    q[1] = lg.random();
    TangentVector_t va, vb, dv;
    JacobianMatrix_t J[2];
    dv.setZero();

    lg.difference (q[0], q[1], va);
    lg.template dDifference<ARG0> (q[0], q[1], J[0]);
    lg.template dDifference<ARG1> (q[0], q[1], J[1]);

    const Scalar eps = 1e-6;
    for (int k = 0; k < 2; ++k) {
      BOOST_TEST_MESSAGE ("Checking J" << k << '\n' << J[k]);
      q_dv[0] = q[0];
      q_dv[1] = q[1];
      // Check J[k]
      for (int i = 0; i < dv.size(); ++i)
      {
        dv[i] = eps;
        lg.integrate (q[k], dv, q_dv[k]);
        lg.difference (q_dv[0], q_dv[1], vb);

        // vb - va ~ J[k] * dv
        TangentVector_t J_dv = J[k].col(i);
        TangentVector_t vb_va = (vb - va) / eps;
        EIGEN_VECTOR_IS_APPROX (vb_va, J_dv, 1e-2);
        dv[i] = 0;
      }
    }

    specificTests(lg);
  }

  template <typename T>
  void specificTests(const T ) const
  {}

  template <typename Scalar, int Options>
  void specificTests(const SpecialEuclideanOperationTpl<3,Scalar,Options>) const
  {
    typedef SE3Tpl<Scalar> SE3;
    typedef SpecialEuclideanOperationTpl<3,Scalar,Options> LG_t;
    typedef typename LG_t::ConfigVector_t ConfigVector_t;
    typedef typename LG_t::JacobianMatrix_t JacobianMatrix_t;
    typedef typename LG_t::ConstQuaternionMap_t ConstQuaternionMap_t;

    LG_t lg;

    ConfigVector_t q[2];
    q[0] = lg.random();
    q[1] = lg.random();
                          
    ConstQuaternionMap_t quat0(q[0].template tail<4>().data()), quat1(q[1].template tail<4>().data());
    JacobianMatrix_t J[2];

    lg.template dDifference<ARG0> (q[0], q[1], J[0]);
    lg.template dDifference<ARG1> (q[0], q[1], J[1]);

    SE3 om0 (typename SE3::Quaternion (q[0].template tail<4>()).matrix(), q[0].template head<3>()),
        om1 (typename SE3::Quaternion (q[1].template tail<4>()).matrix(), q[1].template head<3>()),
        _1m2 (om1.actInv (om0)) ;
    EIGEN_MATRIX_IS_APPROX (J[1] * _1m2.toActionMatrix(), - J[0], 1e-8);
                          
    // Test against SE3::Interpolate
    const Scalar u = 0.3;
    ConfigVector_t q_interp = lg.interpolate(q[0],q[1],u);
    ConstQuaternionMap_t quat_interp(q_interp.template tail<4>().data());
                        
    SE3 M0(quat0,q[0].template head<3>());
    SE3 M1(quat1,q[1].template head<3>());
                          
    SE3 M_u = SE3::Interpolate(M0,M1,u);
    SE3 M_interp(quat_interp,q_interp.template head<3>());
                          
    BOOST_CHECK(M_u.isApprox(M_interp));
  }

  template <typename Scalar, int Options>
    void specificTests(const CartesianProductOperation<
        VectorSpaceOperationTpl<3,Scalar,Options>,
        SpecialOrthogonalOperationTpl<3,Scalar,Options>
        >) const
  {
    typedef SE3Tpl<Scalar> SE3;
    typedef CartesianProductOperation<
      VectorSpaceOperationTpl<3,Scalar,Options>,
      SpecialOrthogonalOperationTpl<3,Scalar,Options>
        > LG_t;
    typedef typename LG_t::ConfigVector_t ConfigVector_t;
    typedef typename LG_t::JacobianMatrix_t JacobianMatrix_t;

    LG_t lg;

    ConfigVector_t q[2];
    q[0] = lg.random();
    q[1] = lg.random();
    JacobianMatrix_t J[2];

    lg.template dDifference<ARG0> (q[0], q[1], J[0]);
    lg.template dDifference<ARG1> (q[0], q[1], J[1]);

    typename SE3::Matrix3
      oR0 (typename SE3::Quaternion (q[0].template tail<4>()).matrix()),
      oR1 (typename SE3::Quaternion (q[1].template tail<4>()).matrix());
    JacobianMatrix_t X (JacobianMatrix_t::Identity());
    X.template bottomRightCorner<3,3>() = oR1.transpose() * oR0;
    EIGEN_MATRIX_IS_APPROX (J[1] * X, - J[0], 1e-8);
  }
};

template<bool around_identity>
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
    TangentVector_t v, dq, dv;
    if(around_identity)
      v.setZero();
    else
      v.setRandom();
    
    dq.setZero();
    dv.setZero();

    ConfigVector_t q_v = lg.integrate (q, v);

    JacobianMatrix_t Jq, Jv;
    lg.dIntegrate_dq (q, v, Jq);
    lg.dIntegrate_dv (q, v, Jv);

    const Scalar eps = 1e-6;
    for (int i = 0; i < v.size(); ++i)
    {
      dq[i] = dv[i] = eps;
      ConfigVector_t q_dq = lg.integrate (q, dq);

      ConfigVector_t q_dq_v = lg.integrate (q_dq, v);
      TangentVector_t Jq_dq = Jq.col(i);
      // q_dv_v - q_v ~ Jq dv
      TangentVector_t dI_dq = lg.difference (q_v, q_dq_v) / eps;
      EIGEN_VECTOR_IS_APPROX (dI_dq, Jq_dq, 1e-2);

      ConfigVector_t q_v_dv = lg.integrate (q, (v+dv).eval());
      TangentVector_t Jv_dv = Jv.col(i);
      // q_v_dv - q_v ~ Jv dv
      TangentVector_t dI_dv = lg.difference (q_v, q_v_dv) / eps;
      EIGEN_VECTOR_IS_APPROX (dI_dv, Jv_dv, 1e-2);

      dq[i] = dv[i] = 0;
    }
  }
};

struct LieGroup_JintegrateJdifference{
  template <typename T>
  void operator()(const T ) const
  {
    typedef typename T::ConfigVector_t ConfigVector_t;
    typedef typename T::TangentVector_t TangentVector_t;
    typedef typename T::JacobianMatrix_t JacobianMatrix_t;

    T lg;
    BOOST_TEST_MESSAGE (lg.name());
    ConfigVector_t qa, qb (lg.nq());
    qa = lg.random();
    TangentVector_t v (lg.nv());
    v.setRandom ();
    lg.integrate(qa, v, qb);

    JacobianMatrix_t Jd_qb, Ji_v;

    lg.template dDifference<ARG1> (qa, qb, Jd_qb);
    lg.template dIntegrate <ARG1> (qa, v , Ji_v );

    BOOST_CHECK_MESSAGE ((Jd_qb * Ji_v).isIdentity(),
        "Jd_qb\n" <<
        Jd_qb << '\n' <<
        "* Ji_v\n" <<
        Ji_v << '\n' <<
        "!= Identity\n" <<
        Jd_qb * Ji_v << '\n');
  }
};

struct LieGroup_JintegrateCoeffWise
{
  template <typename T>
  void operator()(const T ) const
  {
    typedef typename T::ConfigVector_t ConfigVector_t;
    typedef typename T::TangentVector_t TangentVector_t;
    typedef typename T::Scalar Scalar;
    
    T lg;
    ConfigVector_t q = lg.random();
    TangentVector_t dv(TangentVector_t::Zero(lg.nv()));
    
    BOOST_TEST_MESSAGE (lg.name());
    typedef Eigen::Matrix<Scalar,T::NQ,T::NV> JacobianCoeffs;
    JacobianCoeffs Jintegrate(JacobianCoeffs::Zero(lg.nq(),lg.nv()));
    lg.integrateCoeffWiseJacobian(q,Jintegrate);
    JacobianCoeffs Jintegrate_fd(JacobianCoeffs::Zero(lg.nq(),lg.nv()));

    const Scalar eps = 1e-8;
    for (int i = 0; i < lg.nv(); ++i)
    {
      dv[i] = eps;
      ConfigVector_t q_next(ConfigVector_t::Zero(lg.nq()));
      lg.integrate(q, dv,q_next);
      Jintegrate_fd.col(i) = (q_next - q)/eps;
      
      dv[i] = 0;
    }

    EIGEN_MATRIX_IS_APPROX(Jintegrate, Jintegrate_fd, sqrt(eps));
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
  typedef double Scalar;
  enum { Options = 0 };
  
  typedef boost::mpl::vector<  VectorSpaceOperationTpl<1,Scalar,Options>
                             , VectorSpaceOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<3,Scalar,Options>
                             , SpecialEuclideanOperationTpl<2,Scalar,Options>
                             , SpecialEuclideanOperationTpl<3,Scalar,Options>
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<2,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<2,Scalar,Options>
                               >
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<3,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<3,Scalar,Options>
                               >
                             > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_Jdifference());
}

BOOST_AUTO_TEST_CASE ( Jintegrate )
{
  typedef double Scalar;
  enum { Options = 0 };
  
  typedef boost::mpl::vector<  VectorSpaceOperationTpl<1,Scalar,Options>
                             , VectorSpaceOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<3,Scalar,Options>
                             , SpecialEuclideanOperationTpl<2,Scalar,Options>
                             , SpecialEuclideanOperationTpl<3,Scalar,Options>
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<2,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<2,Scalar,Options>
                               >
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<3,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<3,Scalar,Options>
                               >
                             > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_Jintegrate<false>());
  
  // Around identity
  boost::mpl::for_each<Types>(LieGroup_Jintegrate<true>());
}

BOOST_AUTO_TEST_CASE ( Jintegrate_Jdifference )
{
  typedef double Scalar;
  enum { Options = 0 };
  
  typedef boost::mpl::vector<  VectorSpaceOperationTpl<1,Scalar,Options>
                             , VectorSpaceOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<2,Scalar,Options>
                             , SpecialOrthogonalOperationTpl<3,Scalar,Options>
                             , SpecialEuclideanOperationTpl<2,Scalar,Options>
                             , SpecialEuclideanOperationTpl<3,Scalar,Options>
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<2,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<2,Scalar,Options>
                               >
                             , CartesianProductOperation<
                                 VectorSpaceOperationTpl<3,Scalar,Options>,
                                 SpecialOrthogonalOperationTpl<3,Scalar,Options>
                               >
                             > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_JintegrateJdifference());
}

BOOST_AUTO_TEST_CASE(JintegrateCoeffWise)
{
  typedef double Scalar;
  enum { Options = 0 };
  
  typedef boost::mpl::vector<  VectorSpaceOperationTpl<1,Scalar,Options>
  , VectorSpaceOperationTpl<2,Scalar,Options>
  , SpecialOrthogonalOperationTpl<2,Scalar,Options>
  , SpecialOrthogonalOperationTpl<3,Scalar,Options>
  , SpecialEuclideanOperationTpl<2,Scalar,Options>
  , SpecialEuclideanOperationTpl<3,Scalar,Options>
  , CartesianProductOperation<
  VectorSpaceOperationTpl<2,Scalar,Options>,
  SpecialOrthogonalOperationTpl<2,Scalar,Options>
  >
  , CartesianProductOperation<
  VectorSpaceOperationTpl<3,Scalar,Options>,
  SpecialOrthogonalOperationTpl<3,Scalar,Options>
  >
  > Types;
  for (int i = 0; i < 20; ++i)
    boost::mpl::for_each<Types>(LieGroup_JintegrateCoeffWise());
  
  {
    typedef SpecialEuclideanOperationTpl<3,Scalar,Options> LieGroup;
    typedef LieGroup::ConfigVector_t ConfigVector_t;
    LieGroup lg;
    
    ConfigVector_t q = lg.random();
//    TangentVector_t dv(TangentVector_t::Zero(lg.nv()));
    
    typedef Eigen::Matrix<Scalar,LieGroup::NQ,LieGroup::NV> JacobianCoeffs;
    JacobianCoeffs Jintegrate(JacobianCoeffs::Zero(lg.nq(),lg.nv()));
    lg.integrateCoeffWiseJacobian(q,Jintegrate);
    
    
   
  }
}

BOOST_AUTO_TEST_CASE ( test_vector_space )
{
  typedef VectorSpaceOperationTpl<3,double> VSO_t;
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
  VectorSpaceOperationTpl <1,double> vs1;
  Eigen::VectorXd neutral;
  neutral.resize (1);
  neutral.setZero ();
  BOOST_CHECK (vs1.nq () == 1);
  BOOST_CHECK (vs1.nv () == 1);
  BOOST_CHECK (vs1.name () == "R^1");
  BOOST_CHECK (vs1.neutral () == neutral);
  // R^2: neutral = [0, 0]
  VectorSpaceOperationTpl <2,double> vs2;
  neutral.resize (2);
  neutral.setZero ();
  BOOST_CHECK (vs2.nq () == 2);
  BOOST_CHECK (vs2.nv () == 2);
  BOOST_CHECK (vs2.name () == "R^2");
  BOOST_CHECK (vs2.neutral () == neutral);
  // R^3: neutral = [0, 0, 0]
  VectorSpaceOperationTpl <3,double> vs3;
  neutral.resize (3);
  neutral.setZero ();
  BOOST_CHECK (vs3.nq () == 3);
  BOOST_CHECK (vs3.nv () == 3);
  BOOST_CHECK (vs3.name () == "R^3");
  BOOST_CHECK (vs3.neutral () == neutral);
  // SO(2): neutral = [1, 0]
  SpecialOrthogonalOperationTpl<2,double> so2;
  neutral.resize (2); neutral [0] = 1; neutral [1] = 0;
  BOOST_CHECK (so2.nq () == 2);
  BOOST_CHECK (so2.nv () == 1);
  BOOST_CHECK (so2.name () == "SO(2)");
  BOOST_CHECK (so2.neutral () == neutral);
  // SO(3): neutral = [0, 0, 0, 1]
  SpecialOrthogonalOperationTpl<3,double> so3;
  neutral.resize (4); neutral.setZero ();
  neutral [3] = 1;
  BOOST_CHECK (so3.nq () == 4);
  BOOST_CHECK (so3.nv () == 3);
  BOOST_CHECK (so3.name () == "SO(3)");
  BOOST_CHECK (so3.neutral () == neutral);
  // SE(2): neutral = [0, 0, 1, 0]
  SpecialEuclideanOperationTpl <2,double> se2;
  neutral.resize (4); neutral.setZero ();
  neutral [2] = 1;
  BOOST_CHECK (se2.nq () == 4);
  BOOST_CHECK (se2.nv () == 3);
  BOOST_CHECK (se2.name () == "SE(2)");
  BOOST_CHECK (se2.neutral () == neutral);
  // SE(3): neutral = [0, 0, 0, 0, 0, 0, 1]
  SpecialEuclideanOperationTpl <3,double> se3;
  neutral.resize (7); neutral.setZero ();
  neutral [6] = 1;
  BOOST_CHECK (se3.nq () == 7);
  BOOST_CHECK (se3.nv () == 6);
  BOOST_CHECK (se3.name () == "SE(3)");
  BOOST_CHECK (se3.neutral () == neutral);
  // R^2 x SO(2): neutral = [0, 0, 1, 0]
  CartesianProductOperation <VectorSpaceOperationTpl <2,double>,
                             SpecialOrthogonalOperationTpl <2,double> > r2xso2;
  neutral.resize (4); neutral.setZero ();
  neutral [2] = 1;
  BOOST_CHECK (r2xso2.nq () == 4);
  BOOST_CHECK (r2xso2.nv () == 3);
  BOOST_CHECK (r2xso2.name () == "R^2*SO(2)");
  BOOST_CHECK (r2xso2.neutral () == neutral);
  // R^3 x SO(3): neutral = [0, 0, 0, 0, 0, 0, 1]
  CartesianProductOperation <VectorSpaceOperationTpl <3,double>,
                             SpecialOrthogonalOperationTpl <3,double> > r3xso3;
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

template<typename LieGroupCollection>
struct TestLieGroupVariantVisitor
{
  
  typedef LieGroupGenericTpl<LieGroupCollection> LieGroupGeneric;
  typedef typename LieGroupGeneric::ConfigVector_t ConfigVector_t;
  typedef typename LieGroupGeneric::TangentVector_t TangentVector_t;
  
  template<typename Derived>
  void operator() (const LieGroupBase<Derived> & lg) const
  {
    LieGroupGenericTpl<LieGroupCollection> lg_generic(lg.derived());
    test(lg,lg_generic);
  }
  
  template<typename Derived>
  static void test(const LieGroupBase<Derived> & lg,
                   const LieGroupGenericTpl<LieGroupCollection> & lg_generic)
  {
    BOOST_CHECK(lg.nq() == nq(lg_generic));
    BOOST_CHECK(lg.nv() == nv(lg_generic));
    
    BOOST_CHECK(lg.name() == name(lg_generic));
    
    BOOST_CHECK(lg.neutral() == neutral(lg_generic));
    
    typedef typename LieGroupGeneric::ConfigVector_t ConfigVectorGeneric;
    typedef typename LieGroupGeneric::TangentVector_t TangentVectorGeneric;
    
    ConfigVector_t q0 = lg.random();
    TangentVector_t v = TangentVector_t::Random(lg.nv());
    ConfigVector_t qout_ref(lg.nq());
    lg.integrate(q0, v, qout_ref);
    
    ConfigVectorGeneric qout(lg.nq());
    integrate(lg_generic, ConfigVectorGeneric(q0), TangentVectorGeneric(v), qout);
    BOOST_CHECK(qout.isApprox(qout_ref));

    ConfigVector_t q1 (nq(lg_generic));
    random (lg_generic, q1);
    difference(lg_generic, q0, q1, v);
    BOOST_CHECK_EQUAL(lg.distance(q0, q1), distance (lg_generic, q0, q1));
  }
};

BOOST_AUTO_TEST_CASE(test_liegroup_variant)
{
  boost::mpl::for_each<LieGroupCollectionDefault::LieGroupVariant::types>(TestLieGroupVariantVisitor<LieGroupCollectionDefault>());
}

template<typename Lg1, typename Lg2>
void test_liegroup_variant_equal(Lg1 lg1, Lg2 lg2)
{
  typedef LieGroupGenericTpl<LieGroupCollectionDefault> LieGroupGeneric;
  BOOST_CHECK_EQUAL(LieGroupGeneric(lg1), LieGroupGeneric(lg2));
}

template<typename Lg1, typename Lg2>
void test_liegroup_variant_not_equal(Lg1 lg1, Lg2 lg2)
{
  typedef LieGroupGenericTpl<LieGroupCollectionDefault> LieGroupGeneric;
  BOOST_CHECK_PREDICATE( std::not_equal_to<LieGroupGeneric>(),
      (LieGroupGeneric(lg1))(LieGroupGeneric(lg2)) );
}

BOOST_AUTO_TEST_CASE(test_liegroup_variant_comparison)
{
  test_liegroup_variant_equal(
      VectorSpaceOperationTpl<1, double>(),
      VectorSpaceOperationTpl<Eigen::Dynamic, double>(1));
  test_liegroup_variant_not_equal(
      VectorSpaceOperationTpl<1, double>(),
      VectorSpaceOperationTpl<Eigen::Dynamic, double>(2));
}

BOOST_AUTO_TEST_SUITE_END ()
