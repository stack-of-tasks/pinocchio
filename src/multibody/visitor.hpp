//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_visitor_hpp__
#define __se3_visitor_hpp__

#define     BOOST_FUSION_INVOKE_MAX_ARITY 10
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/include/algorithm.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"


namespace boost {
  namespace fusion {

    // Append the element T at the front of boost fusion vector V.
    template<typename T,typename V>
    typename result_of::push_front<V const, T>::type
    append(T const& t,V const& v) { return push_front(v,t); }

    // Append the elements T1 and T2 at the front of boost fusion vector V.
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
                   bf::append2(boost::ref(jmodel),
                               boost::ref(boost::get<typename D::JointDataDerived>(jdataSpec)),
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
    
    
    template<typename Visitor>
    struct JointModelVisitor : public boost::static_visitor<>
    {
      template<typename D>
      void operator() (const JointModelBase<D> & jmodel) const
      {
        bf::invoke(&Visitor::template algo<D>,
                   bf::append(boost::ref(jmodel),
                              static_cast<const Visitor*>(this)->args));
      }
      
      template<typename ArgsTmp>
      static void run(const JointModelVariant & jmodel,
                      ArgsTmp args)
      {
        return boost::apply_visitor( Visitor(args),jmodel );
      }
    };
    
  } // namespace fusion
} // namespace se3

#define JOINT_VISITOR_INIT(VISITOR)					\
  VISITOR( JointDataVariant & jdata,ArgsType args ) : jdata(jdata),args(args) {} \
  using se3::fusion::JointVisitor< VISITOR >::run;			\
  JointDataVariant & jdata;						\
  ArgsType args


#define JOINT_MODEL_VISITOR_INIT(VISITOR)         \
  VISITOR(ArgsType args ) : args(args) {} \
  using se3::fusion::JointModelVisitor< VISITOR >::run;      \
  ArgsType args

#endif // ifndef __se3_visitor_hpp__
