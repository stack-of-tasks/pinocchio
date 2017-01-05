// Copyright (c) 2017, Joseph Mirabel
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

#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;

template <typename T>
void test_lie_group_methods (T & jmodel, typename T::JointDataDerived &)
{
    std::cout << "Testing Joint over " << jmodel.shortname() << std::endl;
    typedef typename T::ConfigVector_t  ConfigVector_t;
    typedef typename T::TangentVector_t TangentVector_t;

    ConfigVector_t  q1(ConfigVector_t::Random (jmodel.nq()));
    TangentVector_t q1_dot(TangentVector_t::Random (jmodel.nv()));
    ConfigVector_t  q2(ConfigVector_t::Random (jmodel.nq()));
    double u = 0.3;
    // se3::Inertia::Matrix6 Ia(se3::Inertia::Random().matrix());
    // bool update_I = false;

    q1 = jmodel.random();
    q2 = jmodel.random();

    // jmodel.calc(jdata, q1, q1_dot);
    // jmodel.calc_aba(jdata, Ia, update_I);

    typedef typename LieGroup<T>::type LieGroupType;
    // LieGroupType lg;

    static Eigen::VectorXd Ones (Eigen::VectorXd::Ones(jmodel.nq()));

    std::string error_prefix("LieGroup ");
    BOOST_CHECK_MESSAGE(jmodel.nq() == LieGroupType::NQ, std::string(error_prefix + " - nq "));
    BOOST_CHECK_MESSAGE(jmodel.nv() == LieGroupType::NV, std::string(error_prefix + " - nv "));

    BOOST_CHECK_MESSAGE(jmodel.integrate(q1,q1_dot).isApprox(LieGroupType::integrate(q1,q1_dot)) ,std::string(error_prefix + " - integrate "));
    BOOST_CHECK_MESSAGE(jmodel.interpolate(q1,q2,u).isApprox(LieGroupType::interpolate(q1,q2,u)) ,std::string(error_prefix + " - interpolate "));
    BOOST_CHECK_MESSAGE(jmodel.randomConfiguration( -1 * Ones, Ones).size()
                        == LieGroupType::randomConfiguration(-1 * Ones, Ones).size(),
                        std::string(error_prefix + " - RandomConfiguration dimensions "));
    BOOST_CHECK_MESSAGE(jmodel.difference(q1,q2).isApprox(LieGroupType::difference(q1,q2)) ,std::string(error_prefix + " - difference "));
    BOOST_CHECK_MESSAGE(fabs(jmodel.distance(q1,q2) - LieGroupType::distance(q1,q2)) < 1e-12 ,std::string(error_prefix + " - distance "));
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

  void operator()(const se3::JointModelComposite & ) const
  {
    // se3::JointModelComposite jmodel(2);
    // jmodel.addJointModel(se3::JointModelRX());
    // jmodel.addJointModel(se3::JointModelRY());

    se3::JointModelComposite jmodel((se3::JointModelRX()));
    jmodel.addJoint(se3::JointModelRY());
    jmodel.setIndexes(0,0,0);

    se3::JointModelComposite::JointDataDerived jdata = jmodel.createData();

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

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_all_joints )
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
  boost::mpl::for_each<Variant::types>(TestJoint());
  // FIXME JointModelComposite does not work.
  // boost::mpl::for_each<JointModelVariant::types>(TestJoint());
}

BOOST_AUTO_TEST_SUITE_END ()
