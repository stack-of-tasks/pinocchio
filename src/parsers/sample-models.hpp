//
// Copyright (c) 2015-2018 CNRS
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
    /** \brief Create a 6-DOF kinematic chain shoulder-elbow-wrist.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     */
    void manipulator(Model & model);
    
#ifdef WITH_HPP_FCL
    /** \brief Create the geometries on top of the kinematic model created by manipulator function.
     *
     * \param model, const, kinematic chain typically produced by the function manipulator(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after manipulator(model).
     */
    void manipulatorGeometries(const Model & model, GeometryModel & geom);
#endif

    /** \brief Create a 28-DOF kinematic chain of a floating humanoid robot.
     * 
     * The kinematic chain has four 6-DOF limbs shoulder-elbow-wrist, one 2-DOF torso, one
     * 2-DOF neck. The float joint is either a JointModelFreeFloating (with nq=7 and nv=6),
     * or a composite joint with a JointModelTranslation
     * and a roll-pitch-yaw joint JointModelSphericalZYX (with total nq=nv=6).
     * Using a free-floating or a composite joint is decided by the boolean usingFF.
     *
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint. This changes the size of the configuration space (35 vs 34).
     */
    void humanoid(Model & model, bool usingFF=true);
    
#ifdef WITH_HPP_FCL
    /** \brief Create the geometries on top of the kinematic model created by humanoid function.
     *
     * \param model, const, kinematic chain typically produced by the function humanoid(model).
     * \warning this method is expecting specific namings of the kinematic chain, use it with care
     * not using after humanoid(model).
     */
    void humanoidGeometries(const Model & model, GeometryModel & geom);
#endif
    
    /** \brief Create a humanoid kinematic tree with 6-DOF limbs and random joint placements.
     *
     * This method is only meant to be used in unittest. Due to random placement and masses,
     * the resulting model is likely to not correspond to any physically-plausible model. 
     * You may want to consider se3::humanoid and se3::humanoidGeometries functions that
     * rather define a plain and non-random model. 
     * \param model: model, typically given empty, where the kinematic chain is added.
     * \param usingFF: if True, implement the chain with a plain JointModelFreeFloating; if False,
     * uses a composite joint translation + roll-pitch-yaw.
     * This changes the size of the configuration space (33 vs 32).
     */
    void humanoidRandom(Model & model, bool usingFF = true);

    /** \brief Create a random humanoid tree with 2d limbs.
     * \ deprecated This function has been replaced by the non-random se3::humanoid function.
     */
    PINOCCHIO_DEPRECATED
    void humanoid2d(Model & model);

    /** \brief Alias of humanoidRandom, for compatibility reasons.
     * \deprecated use se3::humanoid or se3::humanoidRandom instead. 
     */
    PINOCCHIO_DEPRECATED
    inline void humanoidSimple(Model & model, bool usingFF = true) { humanoidRandom(model,usingFF); }
   
  } // namespace buildModels
} // namespace se3

#endif // ifndef __se3_sample_models_hpp__
