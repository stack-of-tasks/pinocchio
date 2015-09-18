//
// Copyright (c) 2015 CNRS
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

      const Model::Index & i = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];
      jmodel.calc(jdata.derived(),q,v);

      // CRBA
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];

      // Jacobian + NLE
      data.v[i] = jdata.v();

      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
      {
        data.oMi[i] = data.liMi[i];
      }

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());

      data.a[i]  = jdata.c() + (data.v[i] ^ jdata.v());
      data.a[i] += data.liMi[i].actInv(data.a[parent]);

      data.f[i] = model.inertias[i]*data.a[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
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
      const Model::Index & i = (Model::Index) jmodel.id();
      const Model::Index & parent = model.parents[i];

      /* F[1:6,i] = Y*S */
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i])
      = jdata.S().transpose()*data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);


      jmodel.jointForce(data.nle)  = jdata.S().transpose()*data.f[i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Eigen::Block<typename Data::Matrix6x> jF
        = data.Fcrb[parent].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
        forceSet::se3Action(data.liMi[i],
                            data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]),
                            jF);

        data.f[parent] += data.liMi[i].act(data.f[i]);
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

    for(Model::Index i=1;i<(Model::Index) model.nbody;++i)
    {
      CATForwardStep::run(model.joints[i],data.joints[i],
                           CATForwardStep::ArgsType(model,data,q,v));
    }

    for(Model::Index i=(Model::Index)(model.nbody-1);i>0;--i)
    {
      CATBackwardStep::run(model.joints[i],data.joints[i],
                            CATBackwardStep::ArgsType(model,data));
    }

  }
} // namespace se3

#endif // ifndef __se3_compute_all_terms_hpp__

