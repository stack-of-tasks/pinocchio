//
// Copyright (c) 2015,2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_visitor_hpp__
#define __pinocchio_visitor_hpp__

#define BOOST_FUSION_INVOKE_MAX_ARITY 10

#include <boost/variant.hpp>
#include <boost/fusion/include/invoke.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>

#include "pinocchio/multibody/joint/joint-base.hpp"

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

namespace pinocchio
{
  namespace fusion
  {
    namespace bf = boost::fusion;
    
    typedef boost::blank NoArg;
    
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
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
      {
        InternalVisitor<JointModelTpl<Scalar,Options,JointCollectionTpl>,NoArg> visitor(jdata);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            typename JointModelBase<JointModelDerived>::JointDataDerived  & jdata,
                            ArgsTmp args)
      {
        InternalVisitor<JointModelDerived,ArgsTmp> visitor(jdata,args);
        return visitor(jmodel.derived());
      }
      
      template<typename JointModelDerived>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            typename JointModelBase<JointModelDerived>::JointDataDerived & jdata)
      {
        InternalVisitor<JointModelDerived,NoArg> visitor(jdata);
        return visitor(jmodel.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            ArgsTmp args)
      {
        ModelOnlyInternalVisitor<ArgsTmp> visitor(args);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
      {
        ModelOnlyInternalVisitor<NoArg> visitor;
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            ArgsTmp args)
      {
        ModelOnlyInternalVisitor<ArgsTmp> visitor(args);
        return visitor(jmodel.derived());
      }
      
      template<typename JointModelDerived>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel)
      {
        ModelOnlyInternalVisitor<NoArg> visitor;
        return visitor(jmodel.derived());
      }
      
    private:
      
      template<typename JointModel, typename ArgType>
      struct InternalVisitor
      : public boost::static_visitor<ReturnType>
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
                                        boost::ref(boost::get<typename JointModelBase<JointModelDerived>::JointDataDerived >(jdata)),
                                        args));
        }
        
        ReturnType operator()(const JointModelVoid) {return;}
        
        JointData & jdata;
        ArgType args;
      };
      
      template<typename JointModel>
      struct InternalVisitor<JointModel,NoArg>
      : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel::JointDataDerived JointData;
        
        InternalVisitor(JointData & jdata)
        : jdata(jdata)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::make_vector(boost::ref(jmodel),
                                            boost::ref(boost::get<typename JointModelBase<JointModelDerived>::JointDataDerived >(jdata)))
                            );
        }
        
        JointData & jdata;
      };
      
      template<typename ArgType, typename Dummy = void>
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
        
        ReturnType operator()(const JointModelVoid) {return;}
        
        ArgType args;
      };
      
      template<typename Dummy>
      struct ModelOnlyInternalVisitor<NoArg,Dummy>
      : public boost::static_visitor<ReturnType>
      {
        ModelOnlyInternalVisitor() {}

        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return JointVisitorDerived::template algo<JointModelDerived>(jmodel);
        }
      };
      
    }; // struct JointVisitorBase
    
  } // namespace fusion
} // namespace pinocchio

#endif // ifndef __pinocchio_visitor_hpp__
