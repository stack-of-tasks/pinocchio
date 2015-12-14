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

#ifndef __se3_kinematics_hpp__
#define __se3_kinematics_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#ifdef WITH_HPP_FCL
  #include "pinocchio/multibody/geometry.hpp"
#endif

namespace se3
{
  inline void geometry(const Model & model,
                       Data & data,
                       const Eigen::VectorXd & q);

  #ifdef WITH_HPP_FCL
  inline void updateCollisionGeometry(const Model & model,
                       Data & data,
                       const GeometryModel& geom,
                       GeometryData& data_geom,
                       const Eigen::VectorXd & q);
  #endif

  inline void kinematics(const Model & model,
                         Data & data,
                         const Eigen::VectorXd & q,
                         const Eigen::VectorXd & v);
  
  inline void dynamics(const Model & model,
                       Data & data,
                       const Eigen::VectorXd & q,
                       const Eigen::VectorXd & v,
                       const Eigen::VectorXd & a);

} // namespace se3 

/* --- Details -------------------------------------------------------------------- */
namespace se3 
{
  struct GeometryStep : public fusion::JointVisitor<GeometryStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const Model::Index,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (GeometryStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::Index i,
                     const Eigen::VectorXd & q)
    {
      using namespace se3;

      jmodel.calc (jdata.derived (), q);

      const Model::Index & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M ();

      if (parent>0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
    }
    
  };

  inline void
  geometry(const Model & model,
           Data & data,
           const Eigen::VectorXd & q)
  {
    for (Model::Index i=1; i < (Model::Index) model.nbody; ++i)
    {
      GeometryStep::run(model.joints[i],
                        data.joints[i],
                        GeometryStep::ArgsType (model,data,i,q)
                        );
    }
  }

#ifdef WITH_HPP_FCL
  struct CollisionGeometryStep : public fusion::JointVisitor<CollisionGeometryStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const se3::GeometryModel &,
                                  se3::GeometryData &,
                                  const Model::Index,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (CollisionGeometryStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & ,
                     se3::JointDataBase<typename JointModel::JointData> & ,
                     const se3::Model & ,
                     se3::Data & data,
                     const se3::GeometryModel & model_geom,
                     se3::GeometryData & data_geom,
                     const Model::Index i,
                     const Eigen::VectorXd & )
    {
      using namespace se3;

      const Model::Index & parent = model_geom.geom_parents[i];
      data_geom.oMg[i] =  (data.oMi[parent] * model_geom.geometryPlacement[i]);
      data_geom.oMg_fcl[i] =  toFclTransform3f(data_geom.oMg[i]);

    }
    
  };

  inline void
  updateCollisionGeometry(const Model & model,
           Data & data,
           const GeometryModel & model_geom,
           GeometryData & data_geom,
           const Eigen::VectorXd & q)
  {
    for (GeometryData::Index i=0; i < (GeometryData::Index) data_geom.model_geom.ngeom; ++i)
    {
      CollisionGeometryStep::run( model.joints[i],
                                  data.joints[i],
                                  CollisionGeometryStep::ArgsType (model,data,model_geom,data_geom,i,q) 
                                );
    }
  }

#endif
  struct KinematicsStep : public fusion::JointVisitor<KinematicsStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
				   se3::Data&,
				   const Model::Index,
				   const Eigen::VectorXd &,
				   const Eigen::VectorXd &
				   > ArgsType;

    JOINT_VISITOR_INIT(KinematicsStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
		    se3::JointDataBase<typename JointModel::JointData> & jdata,
		    const se3::Model& model,
		    se3::Data& data,
		    const Model::Index i,
		    const Eigen::VectorXd & q,
		    const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;
      
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[i];
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
    }

  };

  inline void
  kinematics(const Model & model, Data& data,
	     const Eigen::VectorXd & q,
	     const Eigen::VectorXd & v)
  {
    data.v[0].setZero();

    for( Model::Index i=1; i<(Model::Index) model.nbody; ++i )
      {
	KinematicsStep::run(model.joints[i],data.joints[i],
			    KinematicsStep::ArgsType(model,data,i,q,v));
      }
  }
  
  struct DynamicsStep : public fusion::JointVisitor<DynamicsStep>
  {
    typedef boost::fusion::vector< const se3::Model&,
    se3::Data&,
    const Model::Index,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(DynamicsStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointData> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Model::Index i,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a)
    {
      jmodel.calc(jdata.derived(),q,v);
      
      const Model::Index & parent = model.parents[i];
      data.v[i] = jdata.v();
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      
      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      data.a[i]  = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a[i] += data.liMi[i].actInv(data.a[parent]);
    }
  };
  
  inline void
  dynamics(const Model & model, Data & data,
           const Eigen::VectorXd & q,
           const Eigen::VectorXd & v,
           const Eigen::VectorXd & a)
  {
    data.v[0].setZero();
    data.a[0].setZero();
    
    for( Model::Index i=1; i < (Model::Index) model.nbody; ++i )
    {
      DynamicsStep::run(model.joints[i],data.joints[i],
                        DynamicsStep::ArgsType(model,data,i,q,v,a));
    }
  }
} // namespace se3

#endif // ifndef __se3_kinematics_hpp__

