#ifndef __se3_visitor_hpp__
#define __se3_visitor_hpp__

#define     BOOST_FUSION_INVOKE_MAX_ARITY 10
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/include/algorithm.hpp>


namespace boost {
  namespace fusion {
    template<typename T,typename V>
    typename result_of::push_front<V const, T>::type
    append(T const& t,V const& v) { return push_front(v,t); }

    template<typename T1,typename T2,typename V>
    typename result_of::push_front<typename result_of::push_front<V const, T2>::type const, T1>::type
    append2(T1 const& t1,T2 const& t2,V const& v) { return push_front(push_front(v,t2),t1); }
  }
}


namespace se3
{
  namespace fusion
  {
    namespace bf = boost::fusion;
  
    template<typename Visitor>
    struct JointVisitor : public boost::static_visitor<>
    {
      template<typename D>
      void operator() (const JointModelBase<D> & jmodel) const
      {
	JointDataVariant& jdataSpec = static_cast<const Visitor*>(this)->jdata;

	bf::invoke(&Visitor::template algo<D>,
		   bf::append2(jmodel,
			       boost::ref(boost::get<typename D::JointData&>(jdataSpec)),
			       static_cast<const Visitor*>(this)->args));
      }

      template<typename ArgsTmp>
      static void run(const JointModelVariant & jmodel,
		      JointDataVariant & jdata,
		      ArgsTmp args)
      {
	return boost::apply_visitor( Visitor(jdata,args),jmodel );
      }
    };
  
  } // namespace fusion
} // namespace se3

#define JOINT_VISITOR_INIT(VISITOR)					\
  VISITOR( JointDataVariant & jdata,ArgsType args ) : jdata(jdata),args(args) {} \
  using JointVisitor< VISITOR >::run;					\
  JointDataVariant & jdata;						\
  ArgsType args

#endif // ifndef __se3_visitor_hpp__
