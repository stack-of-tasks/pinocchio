//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_parsers_sdf_hpp__
#define __pinocchio_parsers_sdf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{
  namespace sdf
  {

    ///
    /// \brief Build the model from a URDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The URDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)& contact_models,
               const bool verbose = false);

  } // namespace sdf
} // namespace pinocchio

#include "pinocchio/parsers/sdf/model.hxx"

#endif // ifndef __pinocchio_parsers_sdf_hpp__
