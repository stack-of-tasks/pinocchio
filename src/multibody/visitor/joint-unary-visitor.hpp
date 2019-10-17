//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_multibody_visitior_joint_unary_visitor_hpp__
#define __pinocchio_multibody_visitior_joint_unary_visitor_hpp__

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
    /// \brief Base structure for \b Unary visitation of a JointModel.
    ///        This structure provides runners to call the right visitor according to the number of arguments.
    ///        This should be used when deriving new rigid body algorithms.
    ///
    template<typename JointVisitorDerived, typename ReturnType = void>
    struct JointUnaryVisitorBase
    {
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                            ArgsTmp args)
      {
        InternalVisitorModelAndData<JointModelTpl<Scalar,Options,JointCollectionTpl>,ArgsTmp> visitor(jdata,args);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
      {
        InternalVisitorModelAndData<JointModelTpl<Scalar,Options,JointCollectionTpl>,NoArg> visitor(jdata);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            typename JointModelBase<JointModelDerived>::JointDataDerived & jdata,
                            ArgsTmp args)
      {
        InternalVisitorModelAndData<JointModelDerived,ArgsTmp> visitor(jdata,args);
        return visitor(jmodel.derived());
      }
      
      template<typename JointModelDerived>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            typename JointModelBase<JointModelDerived>::JointDataDerived & jdata)
      {
        InternalVisitorModelAndData<JointModelDerived,NoArg> visitor(jdata);
        return visitor(jmodel.derived());
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsTmp>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                            ArgsTmp args)
      {
        InternalVisitorModel<ArgsTmp> visitor(args);
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      static ReturnType run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
      {
        InternalVisitorModel<NoArg> visitor;
        return boost::apply_visitor(visitor,jmodel);
      }
      
      template<typename JointModelDerived, typename ArgsTmp>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel,
                            ArgsTmp args)
      {
        InternalVisitorModel<ArgsTmp> visitor(args);
        return visitor(jmodel.derived());
      }
      
      template<typename JointModelDerived>
      static ReturnType run(const JointModelBase<JointModelDerived> & jmodel)
      {
        InternalVisitorModel<NoArg> visitor;
        return visitor(jmodel.derived());
      }
      
    private:
      
      template<typename JointModel, typename ArgType>
      struct InternalVisitorModelAndData
      : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel::JointDataDerived JointData;
        
        InternalVisitorModelAndData(JointData & jdata, ArgType args)
        : jdata(jdata), args(args)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::append(boost::ref(jmodel.derived()),
                                       boost::ref(boost::get<typename JointModelBase<JointModelDerived>::JointDataDerived >(jdata)),
                                       args));
        }
        
        ReturnType operator()(const JointModelVoid) {return;}
        
        JointData & jdata;
        ArgType args;
      };
      
      template<typename JointModel>
      struct InternalVisitorModelAndData<JointModel,NoArg>
      : public boost::static_visitor<ReturnType>
      {
        typedef typename JointModel::JointDataDerived JointData;
        
        InternalVisitorModelAndData(JointData & jdata)
        : jdata(jdata)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::make_vector(boost::ref(jmodel.derived()),
                                            boost::ref(boost::get<typename JointModelBase<JointModelDerived>::JointDataDerived >(jdata))));
        }
        
        JointData & jdata;
      };
      
      template<typename ArgType, typename Dummy = void>
      struct InternalVisitorModel
      : public boost::static_visitor<ReturnType>
      {
        InternalVisitorModel(ArgType args)
        : args(args)
        {}
        
        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return bf::invoke(&JointVisitorDerived::template algo<JointModelDerived>,
                            bf::append(boost::ref(jmodel.derived()),
                                       args));
        }
        
        ReturnType operator()(const JointModelVoid) {return;}
        
        ArgType args;
      };
      
      template<typename Dummy>
      struct InternalVisitorModel<NoArg,Dummy>
      : public boost::static_visitor<ReturnType>
      {
        InternalVisitorModel() {}

        template<typename JointModelDerived>
        ReturnType operator()(const JointModelBase<JointModelDerived> & jmodel) const
        {
          return JointVisitorDerived::template algo<JointModelDerived>(jmodel.derived());
        }
      };
    }; // struct JointUnaryVisitorBase
    
    ///
    /// \brief This helper structure is now deprecated and has been replaced by JointUnaryVisitorBase.
    ///
    template<typename JointVisitorDerived, typename ReturnType = void>
    struct PINOCCHIO_DEPRECATED JointVisitorBase
    : JointUnaryVisitorBase<JointVisitorDerived,ReturnType>
    {
      typedef JointUnaryVisitorBase<JointVisitorDerived,ReturnType> Base;
      using Base::run;
    };
    
  } // namespace fusion
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_visitior_joint_unary_visitor_hpp__
