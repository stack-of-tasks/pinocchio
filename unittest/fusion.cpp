// #include "pinocchio/spatial/fwd.hpp"
// #include "pinocchio/spatial/se3.hpp"
// #include "pinocchio/multibody/joint.hpp"
// #include "pinocchio/multibody/model.hpp"

#include <iostream>

#include "pinocchio/tools/timer.hpp"
#include <Eigen/Core>

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
  { std::cout << "h(" << x << "," << y << "," << z << ",a)" << std::endl; return 3; }
};

struct CRTPDerived2 : public CRTPBase<CRTPDerived2>
{
  void f() { std::cout << "f()" << std::endl; }
  int g() { std::cout << "g()" << std::endl; return 1; }
  int h(const double & x) { std::cout << "h(" << x << ")" << std::endl; return 2; }
  int hh(const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & ) 
  { std::cout << "h(" << x << "," << y << "," << z << ",a)" << std::endl; return 3; }
};

// template<typedef Launcher>
// struct LauncherBase : public boost::static_visitor<Launcher::ReturnType>
// {
//   typedef typename Launcher::ReturnType ReturnType;

//   template<typename D>
//   ReturnType operator()( const CRTP<D> & crtp )  const 
//   {
//     return static_cast<D*>(this)->algo(crtp,
// 				       static_cast<D*>(this)->args);
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


using namespace boost::fusion;


template<typename D>
int algo(CRTPBase<D> & crtp, const double & x,const int & y, const Eigen::MatrixXd & z,const TestObj & a) 
{
  return crtp.hh(x,y,z,a);
}

#define CRTP_VARIANT(ReturnType,function)	\
template<typename Args> \
struct Launcher : public boost::static_visitor<ReturnType> \
{ \
  Args args; \
  Launcher(Args args) : args(args) {} \
\
  template<typename D> \
  ReturnType operator()( CRTPBase<D> & dref )  const	\
  { \
    return invoke(&function<D>,join(make_vector(boost::ref(dref)),args));	\
  } \
}; \
template<typename Args> \
ReturnType function( CRTPVariant & crtp, Args args ) \
{ \
  return boost::apply_visitor( Launcher<Args>(args),crtp );  \
}

CRTP_VARIANT(int,algo)


int main()
{
  CRTPDerived d;
  CRTPBase<CRTPDerived> & dref = d;
  CRTPVariant v = d;

  algo(v, make_vector(boost::cref(1.0),boost::cref(1),boost::cref(Eigen::MatrixXd::Zero(3,3)),boost::cref(TestObj(1))) );

  return 0;
}
