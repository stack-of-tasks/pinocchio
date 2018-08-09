//
// Copyright (c) 2015,2018 CNRS
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

#define BOOST_FUSION_INVOKE_MAX_ARITY 10
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/include/algorithm.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"


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

namespace se3
{
  namespace fusion
  {
    namespace bf = boost::fusion;
    
    template<typename JointVisitorDerived, typename ReturnType = void>
    struct JointVisitorBase
    {
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                            ArgsTmp args)
      {
        InternalVisitor<JointModelTpl<Scalar,Options,JointCollectionTpl>,ArgsTmp> visitor(jdata,args);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                      typename JointModelDerived::JointDataDerived & jdata,
                      ArgsTmp args)
      {
        InternalVisitor<JointModelDerived,ArgsTmp> visitor(jdata,args);
        return visitor(jmodel.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            ArgsTmp args)
      {
        ModelOnlyInternalVisitor<ArgsTmp> visitor(args);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            ArgsTmp args)
      {
        ModelOnlyInternalVisitor<ArgsTmp> visitor(args);
        return visitor(jmodel.derived());
      }
      
    private:
      
      template<typename JointModel, typename ArgType>
      struct InternalVisitor : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel::JointDataDerived JointData;
        
        InternalVisitor(JointData & jdata, ArgType args)
        : jdata(jdata), args(args)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::append2(boost::ref(jmodel),
                                        boost::ref(boost::get<typename JointModelDerived::JointDataDerived>(jdata)),
                                        args));
        }
        
        JointData & jdata;
        ArgType args;
      };
      
      template<typename ArgType>
      struct ModelOnlyInternalVisitor : public boost::static_visitor<ReturnType>
      {
        ModelOnlyInternalVisitor(ArgType args)
        : args(args)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::append(boost::ref(jmodel),
                                       args));
        }
        
        ArgType args;
      };
    };
    
  } // namespace fusion
} // namespace se3

#endif // ifndef __se3_visitor_hpp__
