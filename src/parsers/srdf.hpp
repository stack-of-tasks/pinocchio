//
// Copyright (c) 2016-2018 CNRS
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

#ifndef __se3_parser_srdf_hpp__
#define __se3_parser_srdf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"


namespace se3
{
  namespace srdf
  {
    
#ifdef WITH_HPP_FCL
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geomModel Model of the geometries.
    /// \param[out] data_geom Data containing the active collision pairs.
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    void removeCollisionPairsFromSrdf(const Model& model,
                                      GeometryModel & geomModel,
                                      const std::string & filename,
                                      const bool verbose = false) throw (std::invalid_argument);
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geomModel Model of the geometries.
    /// \param[out] data_geom Data containing the active collision pairs.
    /// \param[in] xmlString the SRDF string.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    void removeCollisionPairsFromSrdfString(const Model& model,
                                            GeometryModel & geomModel,
                                            const std::string & xmlString,
                                            const bool verbose = false);
    
#endif // ifdef WITH_HPP_FCL
    

    ///
    /// \brief Get the neutral configuration of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model The Model for which we want the neutral config
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    /// \return The neutral configuration as an eigen vector
    Eigen::VectorXd getNeutralConfigurationFromSrdf(Model & model,
                                                    const std::string & filename,
                                                    const bool verbose = false) throw (std::invalid_argument);


    ///
    /// \brief Load the rotor params of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \param[in] model The Model for which we want the rotor parmeters
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    /// \return Boolean whether it loads or not.    
    bool loadRotorParamsFromSrdf(Model & model,
                                 const std::string & filename,
                                 const bool verbose) throw (std::invalid_argument);
    
  }
} // namespace se3

#include "pinocchio/parsers/srdf.hxx"

#endif // ifndef __se3_parser_srdf_hpp__
