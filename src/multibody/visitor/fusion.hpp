//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_multibody_visitior_fusion_hpp__
#define __pinocchio_multibody_visitior_fusion_hpp__

#define BOOST_FUSION_INVOKE_MAX_ARITY 10

#include <boost/variant/static_visitor.hpp>
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>

namespace boost
{
  namespace fusion
  {

    // Append the element T at the front of boost fusion vector V.
    template<typename T,typename V>
    typename result_of::push_front<V const, T>::type
    append(T const& t,V const& v)
    { return push_front(v,t); }

    // Append the elements T1 and T2 at the front of boost fusion vector V.
    template<typename T1,typename T2,typename V>
    typename result_of::push_front<typename result_of::push_front<V const, T2>::type const, T1>::type
    append2(T1 const& t1,T2 const& t2,V const& v)
    { return push_front(push_front(v,t2),t1); }
    
  }
}

#endif // ifndef __pinocchio_multibody_visitior_fusion_hpp__
