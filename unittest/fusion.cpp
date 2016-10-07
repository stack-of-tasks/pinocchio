//
// Copyright (c) 2015 CNRS
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


#define     BOOST_FUSION_INVOKE_MAX_ARITY 10


// #include "pinocchio/spatial/fwd.hpp"
// #include "pinocchio/spatial/se3.hpp"
// #include "pinocchio/multibody/joint.hpp"
// #include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"
#include <Eigen/Core>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

struct TestObj
{
  int i;
  TestObj() : i(0) { std::cout << "Test()" << std::endl; }
  TestObj(int i) : i(i) { std::cout << "Test(int)" << std::endl; }
  TestObj(const TestObj& clone) : i(clone.i) { std::cout << "Test(clone)" << std::endl; }
};


template<typename D>
struct CRTPBase
{
  D& derived() { return static_cast<D&> (*this); }
  const D& derived() const { return static_cast<const D&> (*this); }

  void f() { static_cast<D*>(this)->f(); }
  int g() { return static_cast<D*>(this)->g(); }
  int h(const double & x) { return static_cast<D*>(this)->h(x); }
  int hh(const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & a) { return static_cast<D*>(this)->hh(x,y,z,a); }
};

struct CRTPDerived : public CRTPBase<CRTPDerived>
{
  void f() { std::cout << "f()" << std::endl; }
  int g() { std::cout << "g()" << std::endl; return 1; }
  int h(const double & x) { std::cout << "h(" << x << ")" << std::endl; return 2; }
  int hh(const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & ) 
  { std::cout << "hh(" << x << "," << y << "," << z << ",a)" << std::endl; return 3; }
};

struct CRTPDerived2 : public CRTPBase<CRTPDerived2>
{
  void f() { std::cout << "f()" << std::endl; }
  int g() { std::cout << "g()" << std::endl; return 1; }
  int h(const double & x) { std::cout << "h(" << x << ")" << std::endl; return 2; }
  int hh(const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & ) 
  { std::cout << "hh(" << x << "," << y << "," << z << ",a)" << std::endl; return 3; }
};

// template<typedef Launcher>
// struct LauncherBase : public boost::static_visitor<Launcher::ReturnType>
// {
//   typedef typename Launcher::ReturnType ReturnType;

//   template<typename D>
//   ReturnType operator()( const CRTP<D> & crtp )  const 
//   {
//     return static_cast<D*>(this)->algo(crtp,
//               static_cast<D*>(this)->args);
//   }

//   static 
// };

#include <boost/variant.hpp>
typedef boost::variant<CRTPDerived,CRTPDerived2> CRTPVariant;

#include <boost/fusion/include/sequence.hpp>
#include <boost/fusion/include/make_vector.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/view/joint_view.hpp>
#include <boost/fusion/include/joint_view.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/container.hpp>


namespace bf = boost::fusion;

struct Launcher : public boost::static_visitor<int>
{

  typedef bf::vector<const double &,const int &, const Eigen::MatrixXd &,
         const Eigen::MatrixXd &,const Eigen::MatrixXd &,const TestObj &> Args;
  Args args;

  Launcher(Args args) : args(args) {}

  template<typename D>
  int operator() ( CRTPBase<D> & dref )  const
  {
    return bf::invoke(&Launcher::algo<D>,bf::push_front(args,boost::ref(dref)));
  }

  static int run(CRTPVariant & crtp, Args args )
  {
    return boost::apply_visitor( Launcher(args),crtp );
  }

  template<typename D>
  static int algo(CRTPBase<D> & crtp, const double & x,const int & y, const Eigen::MatrixXd & z,
      const Eigen::MatrixXd & ,const Eigen::MatrixXd & ,const TestObj & a) 
  {
    return crtp.hh(x,y,z,a);
  }
};

namespace boost {
  namespace fusion {
    template<typename T,typename V>
    typename result_of::push_front<V const, T>::type
    append(T const& t,V const& v) { return push_front(v,t); }

    template<typename T1,typename T2,typename V>
    typename result_of::push_front<typename result_of::push_front<V const, T2>::type const, T1>::type
    append2(T1 const& t1,T2 const& t2,V const& v) { return push_front(push_front(v,t2),t1); }


    // template<typename t1,typename t2,typename v>


    // typename result_of::push_front<Sequence, T>::type

    // res append2(t1 a1,t2 a2,v a3) { return push_front(push_front(a3,a2),a1); }
  }}


BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_fusion )
{
  CRTPDerived d;
  //CRTPBase<CRTPDerived> & dref = d;
  CRTPVariant v = d;

  //(CRTPBase<D> & crtp, const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & a)
  

  //Args args(1.0,1,Eigen::MatrixXd::Zero(3,3),TestObj(1));
  Launcher::run(v,  Launcher::Args(1.0,1,Eigen::MatrixXd::Zero(3,3),Eigen::MatrixXd::Zero(3,3),
           Eigen::MatrixXd::Zero(3,3),TestObj(1)) );

  int i,j; double k;
  bf::vector<int&> arg = bf::make_vector(boost::ref(j));

  bf::vector<double &,int &> arg1 = bf::append(boost::ref(k),arg);
  bf::vector<int &,double &,int &> arg11 = bf::append(boost::ref(i),arg1);

  bf::vector<int &,double &,int &> arg2 = bf::append2(boost::ref(i),boost::ref(k),arg);
    //bf::push_front(bf::push_front(arg1,boost::ref(k)),boost::ref(j));
}

BOOST_AUTO_TEST_SUITE_END ()

