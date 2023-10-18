//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_multibody_visitior_joint_binary_visitor_hpp__
#define __pinocchio_multibody_visitior_joint_binary_visitor_hpp__

#include <boost/variant.hpp>

#include "pinocchio/multibody/joint/joint-base.hpp"
#include "pinocchio/multibody/visitor/fusion.hpp"

namespace pinocchio
{
  namespace fusion
  {
    namespace bf = boost::fusion;
    
    typedef boost::blank NoArg;
    
    ///
    /// \brief Base structure for \b Binary  visitation of two JointModels.
    ///        This structure provides runners to call the right visitor according to the number of arguments.
    ///        This should be used when deriving new rigid body algorithms.
    ///
    template<typename JointVisitorDerived, typename ReturnType = void>
    struct JointBinaryVisitorBase
    {
      
      template<typename JointModelDerived1, typename JointModelDerived2, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived1> & jmodel1,
                            const JointModelBase<JointModelDerived2> & jmodel2,
                            typename JointModelBase<JointModelDerived1>::JointDataDerived & jdata1,
                            typename JointModelBase<JointModelDerived2>::JointDataDerived & jdata2,
                            ArgsTmp args)
      {
        InternalVisitorModelAndData<JointModelDerived1,JointModelDerived2,ArgsTmp> visitor(jdata1,jdata2,args);
        return visitor(jmodel1.derived(),jmodel2.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel1,
                            const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel2,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata1,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata2,
                            ArgsTmp args)
      {
        typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModel;
        InternalVisitorModelAndData<JointModel,JointModel,ArgsTmp> visitor(jdata1,jdata2,args);
        return boost::apply_visitor(visitor,jmodel1,jmodel2);
      }
      
      template<typename JointModelDerived1, typename JointModelDerived2>
      static ReturnType run(const JointModelBase<JointModelDerived1> & jmodel1,
                            const JointModelBase<JointModelDerived2> & jmodel2,
                            typename JointModelBase<JointModelDerived1>::JointDataDerived & jdata1,
                            typename JointModelBase<JointModelDerived2>::JointDataDerived & jdata2)
      {
        InternalVisitorModelAndData<JointModelDerived1,JointModelDerived2,NoArg> visitor(jdata1,jdata2);
        return visitor(jmodel1.derived(),jmodel2.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel1,
                            const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel2,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata1,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata2)
      {
        typedef JointModelTpl<Scalar,Options,JointCollectionTpl> JointModel;
        InternalVisitorModelAndData<JointModel,JointModel,NoArg> visitor(jdata1,jdata2);
        return boost::apply_visitor(visitor,jmodel1,jmodel2);
      }
      
      template<typename JointModelDerived1, typename JointModelDerived2, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived1> & jmodel1,
                            const JointModelBase<JointModelDerived2> & jmodel2,
                            ArgsTmp args)
      {
        InternalVisitorModel<ArgsTmp> visitor(args);
        return visitor(jmodel1.derived(),jmodel2.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel1,
                            const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel2,
                            ArgsTmp args)
      {
        InternalVisitorModel<ArgsTmp> visitor(args);
        return boost::apply_visitor(visitor,jmodel1,jmodel2);
      }
      
      template<typename JointModelDerived1, typename JointModelDerived2>
      static ReturnType run(const JointModelBase<JointModelDerived1> & jmodel1,
                            const JointModelBase<JointModelDerived2> & jmodel2)
      {
        InternalVisitorModel<NoArg> visitor;
        return visitor(jmodel1.derived(),jmodel2.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel1,
                            const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel2)
      {
        InternalVisitorModel<NoArg> visitor;
        return boost::apply_visitor(visitor,jmodel1,jmodel2);
      }
      
    private:
      
      template<typename JointModel1, typename JointModel2, typename ArgType>
      struct InternalVisitorModelAndData
      : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel1::JointDataDerived JointData1;
        typedef typename JointModel2::JointDataDerived JointData2;
        
        InternalVisitorModelAndData(JointData1 & jdata1,
                                    JointData2 & jdata2,
                                    ArgType args)
        : jdata1(jdata1)
        , jdata2(jdata2)
        , args(args)
        {}
        
        template<typename JointModelDerived1, typename JointModelDerived2>
        ReturnType operator()(const JointModelBase<JointModelDerived1> & jmodel1,
                              const JointModelBase<JointModelDerived2> & jmodel2) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived1,JointModelDerived2>,
                            bf::append(boost::ref(jmodel1.derived()),
                                       boost::ref(jmodel2.derived()),
                                       boost::ref(boost::get<typename JointModelBase<JointModelDerived1>::JointDataDerived >(jdata1)),
                                       boost::ref(boost::get<typename JointModelBase<JointModelDerived2>::JointDataDerived >(jdata2)),
                                       args));
        }
        
        JointData1 & jdata1;
        JointData2 & jdata2;
        
        ArgType args;
      };
      
      template<typename JointModel1, typename JointModel2>
      struct InternalVisitorModelAndData<JointModel1,JointModel2,NoArg>
      : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel1::JointDataDerived JointData1;
        typedef typename JointModel2::JointDataDerived JointData2;
        
        InternalVisitorModelAndData(JointData1 & jdata1, JointData2 & jdata2)
        : jdata1(jdata1)
        , jdata2(jdata2)
        {}
        
        template<typename JointModelDerived1, typename JointModelDerived2>
        ReturnType operator()(const JointModelBase<JointModelDerived1> & jmodel1,
                              const JointModelBase<JointModelDerived2> & jmodel2) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived1,JointModelDerived2>,
                            bf::make_vector(boost::ref(jmodel1.derived()),
                                            boost::ref(jmodel2.derived()),
                                            boost::ref(boost::get<typename JointModelBase<JointModelDerived1>::JointDataDerived >(jdata1)),
                                            boost::ref(boost::get<typename JointModelBase<JointModelDerived2>::JointDataDerived >(jdata2))));
        }
        
        JointData1 & jdata1;
        JointData2 & jdata2;
      };
      
      template<typename ArgType, typename Dummy = void>
      struct InternalVisitorModel
      : public boost::static_visitor<ReturnType>
      {
        InternalVisitorModel(ArgType args)
        : args(args)
        {}
        
        template<typename JointModel1, typename JointModel2>
        ReturnType operator()(const JointModelBase<JointModel1> & jmodel1,
                              const JointModelBase<JointModel2> & jmodel2) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModel1,JointModel2>,
                            bf::append(boost::ref(jmodel1.derived()),
                                       boost::ref(jmodel2.derived()),
                                       args));
        }
        
        ArgType args;
      };
      
      template<typename Dummy>
      struct InternalVisitorModel<NoArg,Dummy>
      : public boost::static_visitor<ReturnType>
      {
        InternalVisitorModel() {}
        
        template<typename JointModel1, typename JointModel2>
        ReturnType operator()(const JointModelBase<JointModel1> & jmodel1,
                              const JointModelBase<JointModel2> & jmodel2) const
        {
          return JointVisitorDerived::
          template algo<JointModel1,JointModel2>(jmodel1.derived(),
                                                 jmodel2.derived());
        }
      };
      
    }; // struct JointBinaryVisitorBase
    
  } // namespace fusion
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_visitior_joint_binary_visitor_hpp__
