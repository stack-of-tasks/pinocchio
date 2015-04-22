#ifndef __se3_compute_all_terms_hpp__
#define __se3_compute_all_terms_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/act-on-set.hpp"

#include <iostream>

namespace se3
{
  inline void
  computeAllTerms(const Model & model,
                  Data & data,
                  const Eigen::VectorXd & q,
                  const Eigen::VectorXd & v);

} // namespace se3

/* --- Details -------------------------------------------------------------------- */
namespace se3
{
  struct CATForwardStep : public fusion::JointVisitor<CATForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;

    JOINT_VISITOR_INIT(CATForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;

      const typename JointModel::Index & i = jmodel.id();
      const Model::Index & parent = model.parents[(size_t) i];
      jmodel.calc(jdata.derived(),q,v);

      // CRBA
      data.liMi[(size_t) i] = model.jointPlacements[(size_t) i]*jdata.M();
      data.Ycrb[(size_t) i] = model.inertias[(size_t) i];

      // Jacobian + NLE
      data.v[(size_t) i] = jdata.v();

      if(parent>0)
      {
        data.oMi[(size_t) i] = data.oMi[(size_t) parent]*data.liMi[(size_t) i];
        data.v[(size_t) i] += data.liMi[(size_t) i].actInv(data.v[(size_t) parent]);
      }
      else
      {
        data.oMi[(size_t) i] = data.liMi[(size_t) i];
      }

      data.J.block(0,jmodel.idx_v(),6,jmodel.nv()) = data.oMi[(size_t) i].act(jdata.S());

      data.a[(size_t) i]  = jdata.c() + (data.v[(size_t) i] ^ jdata.v());
      data.a[(size_t) i] += data.liMi[(size_t) i].actInv(data.a[(size_t) parent]);

      data.f[(size_t) i] = model.inertias[(size_t) i]*data.a[(size_t) i] + model.inertias[(size_t) i].vxiv(data.v[(size_t) i]); // -f_ext
    }

  };

  struct CATBackwardStep : public fusion::JointVisitor<CATBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &>  ArgsType;

    JOINT_VISITOR_INIT(CATBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointData> & jdata,
                     const Model & model,
                     Data & data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      const Model::Index & i = jmodel.id();
      const Model::Index & parent = model.parents[(size_t) i];

      /* F[1:6,i] = Y*S */
      data.Fcrb[(std::size_t)i].block<6,JointModel::NV>(0,jmodel.idx_v()) = data.Ycrb[(size_t) i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[(size_t) i])
      = jdata.S().transpose()*data.Fcrb[(size_t) i].block(0,jmodel.idx_v(),6,data.nvSubtree[(size_t) i]);


      jmodel.jointForce(data.nle)  = jdata.S().transpose()*data.f[(size_t) i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[(size_t) parent] += data.liMi[(size_t) i].act(data.Ycrb[(size_t) i]);

        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Eigen::Block<typename Data::Matrix6x> jF
        = data.Fcrb[(std::size_t)parent].block(0,jmodel.idx_v(),6,data.nvSubtree[(size_t) i]);
        forceSet::se3Action(data.liMi[(size_t) i],
                            data.Fcrb[(size_t) i].block(0,jmodel.idx_v(),6,data.nvSubtree[(size_t) i]),
                            jF);

        data.f[(size_t) parent] += data.liMi[(size_t) i].act(data.f[(size_t) i]);
      }
    }
  };

  inline void
  computeAllTerms(const Model & model,
                  Data & data,
                  const Eigen::VectorXd & q,
                  const Eigen::VectorXd & v)
  {
    data.v[0].setZero ();
    data.a[0] = -model.gravity;

    for(size_t i=1;i<(size_t) model.nbody;++i)
    {
      CATForwardStep::run(model.joints[i],data.joints[i],
                           CATForwardStep::ArgsType(model,data,q,v));
    }

    for(size_t i=(size_t)(model.nbody-1);i>0;--i)
    {
      CATBackwardStep::run(model.joints[i],data.joints[i],
                            CATBackwardStep::ArgsType(model,data));
    }

  }
} // namespace se3

#endif // ifndef __se3_compute_all_terms_hpp__

