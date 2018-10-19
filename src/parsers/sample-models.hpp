//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_sample_models_hpp__
#define __se3_sample_models_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

namespace se3
{
  namespace buildModels
  {

    void humanoid2d(Model& model);
    void humanoidSimple(Model& model, bool usingFF = true);

    /** \brief Create a 6-DOF kinematic chain shoulder-elbow-wrist.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     */
    void manipulator(Model& model);
    /** \brief Create the geometries on top of the kinematic model created by manipulator function.
     *
     * \param model, const, kinematic chain typically produced by the function manipulator(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after manipulator(model).
     */
    void manipulatorGeometries(const Model& model, GeometryModel & geom);

    /** \brief Create a 28-DOF kinematic chain of a floating humanoid robot.
     * 
     * The kinematic chain has 4 limbs shoulder-elbow-wrist, one 2-dof torso, one
     * 2-dof neck. The float joint is either a free-float joint JointModelFreeFloating
     * (with nq=7 and nv=6), or a composite joint with 3 prismatic and 1 roll-pitch-yaw.
     * Using a free-floating or a composite joint is decided by the boolean usingFF.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint. This changes the size of the configuration space (35 vs 34).
     */
    void humanoid(Model& model,bool usingFF=true);
    /** \brief Create the geometries on top of the kinematic model created by humanoid function.
     *
     * \param model, const, kinematic chain typically produced by the function humanoid(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after humanoid(model).
     */
    void humanoidGeometries(const Model& model, GeometryModel & geom);

  } // namespace buildModels
} // namespace se3

#endif // ifndef __se3_sample_models_hpp__
