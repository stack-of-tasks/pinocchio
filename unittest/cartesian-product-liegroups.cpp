//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/liegroup-collection.hpp"
#include "pinocchio/multibody/liegroup/liegroup-generic.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product-variant.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product.hpp"

#include "pinocchio/multibody/joint/joint-generic.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include <boost/algorithm/string.hpp>

using namespace pinocchio;

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


template<typename Scalar, int Options, template<typename S, int O> class LieGroupCollectionTpl>
struct TestCartesianProduct
{
  
  typedef LieGroupCollectionTpl<Scalar,Options> LieGroupCollection;
  
  typedef LieGroupGenericTpl<LieGroupCollection> LieGroupGeneric;
  typedef typename LieGroupGeneric::ConfigVector_t ConfigVector_t;
  typedef typename LieGroupGeneric::TangentVector_t TangentVector_t;
  
  typedef CartesianProductOperationVariantTpl<Scalar, Options, LieGroupCollectionTpl > CartesianProduct;
  
  template<typename Derived>
  void operator() (const LieGroupBase<Derived> & lg) const
  {
    LieGroupGenericTpl<LieGroupCollection> lg_generic(lg.derived());
    CartesianProduct cp(lg_generic);
    test(lg,cp);
    
    CartesianProduct cp2;
    cp2.append(lg);
    BOOST_CHECK(cp == cp2);
  }
  
  template<typename LieGroup>
  static void test(const LieGroupBase<LieGroup> & lg,
                   const CartesianProduct & cp)
  {
    BOOST_CHECK(lg.nq() == cp.nq());
    BOOST_CHECK(lg.nv() == cp.nv());
    
    std::cout << "name: " << cp.name() << std::endl;
    
    BOOST_CHECK(lg.neutral() == cp.neutral());

    typedef typename LieGroup::ConfigVector_t ConfigVector;
    typedef typename LieGroup::TangentVector_t TangentVector;
    typedef typename LieGroup::JacobianMatrix_t JacobianMatrix;

    ConfigVector q0 = lg.random();
    ConfigVector q1 = lg.random();
    TangentVector v = TangentVector_t::Random(lg.nv());
    ConfigVector qout_ref(lg.nq()), qout(lg.nq());
    lg.integrate(q0, v, qout_ref);
    cp.integrate(q0, v, qout);
    
    BOOST_CHECK(qout.isApprox(qout_ref));
    
    TangentVector v_diff_ref(lg.nv()), v_diff(lg.nv());
    lg.difference(q0,q1,v_diff_ref);
    cp.difference(q0,q1,v_diff);
    
    BOOST_CHECK(v_diff_ref.isApprox(v_diff));
    BOOST_CHECK_EQUAL(lg.squaredDistance(q0, q1), cp.squaredDistance(q0, q1));
    BOOST_CHECK_EQUAL(lg.distance(q0, q1), cp.distance(q0, q1));
    
    JacobianMatrix
    J_ref(JacobianMatrix::Zero(lg.nv(),lg.nv())),
    J(JacobianMatrix::Zero(lg.nv(),lg.nv()));
    
    lg.dDifference(q0, q1, J_ref, ARG0);
    cp.dDifference(q0, q1, J, ARG0);
    
    BOOST_CHECK(J.isApprox(J_ref));
    
    lg.dDifference(q0, q1, J_ref, ARG1);
    cp.dDifference(q0, q1, J, ARG1);
    
    BOOST_CHECK(J.isApprox(J_ref));
    
    lg.dIntegrate(q0, v, J_ref, ARG0);
    cp.dIntegrate(q0, v, J, ARG0);
    
    BOOST_CHECK(J.isApprox(J_ref));
    
    lg.dIntegrate(q0, v, J_ref, ARG1);
    cp.dIntegrate(q0, v, J, ARG1);
    
    BOOST_CHECK(J.isApprox(J_ref));
    
    BOOST_CHECK(cp.isSameConfiguration(q0,q0));
    ConfigVector q_rand;
    cp.random(q_rand);
    ConfigVector q_rand_copy = q_rand;
    
    lg.normalize(q_rand_copy);
    cp.normalize(q_rand);
    BOOST_CHECK(q_rand.isApprox(q_rand_copy));
    
    const ConfigVector lb(-ConfigVector::Ones(lg.nq()));
    const ConfigVector ub( ConfigVector::Ones(lg.nq()));
    
    cp.randomConfiguration(lb, ub, q_rand);
  }
};

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_cartesian_product_with_liegroup_variant)
{
  boost::mpl::for_each<LieGroupCollectionDefault::LieGroupVariant::types>(TestCartesianProduct<double,0,LieGroupCollectionDefaultTpl>());
}

BOOST_AUTO_TEST_CASE(test_cartesian_product_vs_cartesian_product_variant)
{
  typedef SpecialEuclideanOperationTpl<3,double,0> SE3;
  typedef VectorSpaceOperationTpl<3,double,0> Rn;
  
  typedef CartesianProductOperation<SE3, Rn> CPRef;
  typedef CartesianProductOperationVariantTpl<double, 0, LieGroupCollectionDefaultTpl > CP;
  
  SE3 lg1; Rn lg2;
  typedef LieGroupGenericTpl<CP::LieGroupCollection> LieGroupGeneric;
  LieGroupGeneric lg1_variant(lg1);
  LieGroupGeneric lg2_variant(lg2);
  
  CP cartesian_product(lg1_variant,lg2_variant);
  CP cartesian_product2(lg1_variant); cartesian_product2.append(lg2_variant);
  std::cout << "cartesian_product: " << cartesian_product << std::endl;
  
  BOOST_CHECK(cartesian_product == cartesian_product2);
  CPRef cartesian_product_ref;
  
  TestCartesianProduct<double,0,LieGroupCollectionDefaultTpl>::test(cartesian_product_ref,
                                                                    cartesian_product);
}

BOOST_AUTO_TEST_SUITE_END()
