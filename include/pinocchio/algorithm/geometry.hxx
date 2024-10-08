//
// Copyright (c) 2015-2022 CNRS INRIA
//

#ifndef __pinocchio_algo_geometry_hxx__
#define __pinocchio_algo_geometry_hxx__

#include <boost/foreach.hpp>
#include <sstream>

namespace pinocchio
{
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  /* --- GEOMETRY PLACEMENTS -------------------------------------------------------- */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  inline void updateGeometryPlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");

    forwardKinematics(model, data, q);
    updateGeometryPlacements(model, data, geom_model, geom_data);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void updateGeometryPlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const GeometryModel & geom_model,
    GeometryData & geom_data)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    assert(model.check(data) && "data is not consistent with model.");

    for (GeomIndex i = 0; i < (GeomIndex)geom_model.ngeoms; ++i)
    {
      const Model::JointIndex joint_id = geom_model.geometryObjects[i].parentJoint;
      if (joint_id > 0)
        geom_data.oMg[i] = (data.oMi[joint_id] * geom_model.geometryObjects[i].placement);
      else
        geom_data.oMg[i] = geom_model.geometryObjects[i].placement;
    }
  }

  /* --- APPEND GEOMETRY MODEL ----------------------------------------------------------- */

  inline void appendGeometryModel(GeometryModel & geom_model1, const GeometryModel & geom_model2)
  {
    assert(geom_model1.ngeoms == geom_model1.geometryObjects.size());
    Index nGeom1 = geom_model1.geometryObjects.size();
    Index nColPairs1 = geom_model1.collisionPairs.size();
    assert(geom_model2.ngeoms == geom_model2.geometryObjects.size());
    Index nGeom2 = geom_model2.geometryObjects.size();
    Index nColPairs2 = geom_model2.collisionPairs.size();

    /// Append the geometry objects and geometry positions
    geom_model1.geometryObjects.insert(
      geom_model1.geometryObjects.end(), geom_model2.geometryObjects.begin(),
      geom_model2.geometryObjects.end());
    geom_model1.ngeoms += nGeom2;

    /// 1. copy the collision pairs and update geom_data1 accordingly.
    geom_model1.collisionPairs.reserve(nColPairs1 + nColPairs2 + nGeom1 * nGeom2);
    for (Index i = 0; i < nColPairs2; ++i)
    {
      const CollisionPair & cp = geom_model2.collisionPairs[i];
      geom_model1.collisionPairs.push_back(CollisionPair(cp.first + nGeom1, cp.second + nGeom1));
    }

    /// 2. add the collision pairs between geom_model1 and geom_model2.
    for (Index i = 0; i < nGeom1; ++i)
    {
      Index parent_i = geom_model1.geometryObjects[i].parentJoint;
      for (Index j = nGeom1; j < nGeom1 + nGeom2; ++j)
      {
        if (parent_i != geom_model1.geometryObjects[j].parentJoint)
          geom_model1.collisionPairs.push_back(CollisionPair(i, j));
      }
    }
  }

} // namespace pinocchio

#endif // ifnded __pinocchio_algo_geometry_hxx__
