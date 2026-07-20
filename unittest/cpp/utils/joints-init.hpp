//
// Copyright (c) 2025 INRIA
//

#include <vector>

#include "pinocchio/multibody/joint.hpp"

namespace pinocchio
{
  template<typename JointModel_>
  struct JointModelWithParameters
  {
    typedef typename JointModel_::ConfigVector_t ConfigVector;
    JointModel_ jmodel;
    ConfigVector lb;
    ConfigVector ub;
  };

  template<typename JointModel_>
  struct init
  {
    static JointModelWithParameters<JointModel_> run()
    {
      JointModel_ jmodel;
      jmodel.setIndexes(0, 0, 0);

      // Default generic bounds
      typename JointModel_::ConfigVector_t lb =
        JointModel_::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel_::ConfigVector_t ub =
        JointModel_::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelRevoluteUnboundedUnalignedTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelPrismaticUnalignedTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelHelicalUnalignedTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef typename JointModel::Vector3 Vector3;
      JointModel jmodel(Vector3::Random().normalized());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelUniversalTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelUniversalTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      JointModel jmodel(XAxis::vector(), YAxis::vector());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options, int axis>
  struct init<pinocchio::JointModelHelicalTpl<Scalar, Options, axis>>
  {
    typedef pinocchio::JointModelHelicalTpl<Scalar, Options, axis> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      JointModel jmodel(static_cast<Scalar>(0.5));

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelEllipsoidTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelEllipsoidTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      JointModel jmodel(Scalar(0.01), Scalar(0.02), Scalar(0.03));

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  struct init<pinocchio::JointModelTpl<Scalar, Options, JointCollection>>
  {
    typedef pinocchio::JointModelTpl<Scalar, Options, JointCollection> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
      JointModel jmodel((JointModelRX()));

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  struct init<pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollection>>
  {
    typedef pinocchio::JointModelCompositeTpl<Scalar, Options, JointCollection> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
      typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 1> JointModelRY;
      JointModel jmodel((JointModelRX()));
      jmodel.addJoint(JointModelRY());

      jmodel.setIndexes(0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);
      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollection>
  struct init<pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection>>
  {
    typedef pinocchio::JointModelMimicTpl<Scalar, Options, JointCollection> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 0> JointModelRX;
      auto jmodelParams_ref = init<JointModelRX>::run();
      ;
      JointModelRX jmodel_ref = jmodelParams_ref.jmodel;

      JointModel jmodel(jmodel_ref, 1., 0.);
      jmodel.setIndexes(1, 0, 0, 0);
      typename JointModel::ConfigVector_t lb =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), -1.0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);

      return {jmodel, lb, ub};
    }
  };

  template<typename Scalar, int Options>
  struct init<pinocchio::JointModelSplineTpl<Scalar, Options>>
  {
    typedef pinocchio::JointModelSplineTpl<Scalar, Options> JointModel;
    typedef JointModelWithParameters<JointModel> ReturnType;

    static ReturnType run()
    {
      std::vector<pinocchio::SE3> ctrlFrames;
      for (int k = 0; k < 5; k++)
        ctrlFrames.push_back(SE3::Random());

      JointModel jmodel = JointModelSplineBuilder()
                            .withControlFrameVector(ctrlFrames)
                            .withDegree(3)
                            .withOpenUniformKnots(0., 1.)
                            .build();
      jmodel.setIndexes(0, 0, 0);

      typename JointModel::ConfigVector_t lb = JointModel::ConfigVector_t::Constant(jmodel.nq(), 0);
      typename JointModel::ConfigVector_t ub =
        JointModel::ConfigVector_t::Constant(jmodel.nq(), 1.0);
      return {jmodel, lb, ub};
    }
  };
} // namespace pinocchio
