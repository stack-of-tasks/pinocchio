//
// Copyright (c) 2016 CNRS
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
#include <iostream>

// Read XML file with boost
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <boost/foreach.hpp>


namespace se3
{
  namespace srdf
  {
    
#ifdef WITH_HPP_FCL
    ///
    /// \brief Deactive all possible collision pairs mentioned in the SRDF file.
    ///        It throws if the SRDF file is incorrect.
    ///
    /// \ingroup parsers_group
    /// 
    /// \param[in] model Model of the kinematic tree.
    /// \param[in] geomModel Model of the geometries.
    /// \param[out] data_geom Data containing the active collision pairs.
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode (print removed collision pairs and undefined link inside the model).
    ///
    inline void removeCollisionPairsFromSrdf(const Model& model,
                                             GeometryModel & geomModel,
                                             const std::string & filename,
                                             const bool verbose) throw (std::invalid_argument)
    {
      // Check extension
      const std::string extension = filename.substr(filename.find_last_of('.')+1);
      if (extension != "srdf")
      {
        const std::string exception_message (filename + " does not have the right extension.");
        throw std::invalid_argument(exception_message);
      }
      
      // Open file
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
      
      // Read xml stream
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(srdf_stream, pt);
      
      // Iterate over collision pairs
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot"))
      {
        if (v.first == "disable_collisions")
        {
          const std::string link1 = v.second.get<std::string>("<xmlattr>.link1");
          const std::string link2 = v.second.get<std::string>("<xmlattr>.link2");
          
          // Check first if the two bodies exist in model
          if (!model.existBodyName(link1) || !model.existBodyName(link2))
          {
            if (verbose)
              std::cout << "It seems that " << link1 << " or " << link2 << " do not exist in model. Skip." << std::endl;
            continue;
          }
          
          const Model::JointIndex frame_id1 = model.getBodyId(link1);
          const Model::JointIndex joint_id1 = model.frames[frame_id1].parent;
          const Model::JointIndex frame_id2 = model.getBodyId(link2);
          const Model::JointIndex joint_id2 = model.frames[frame_id2].parent;

          // Malformed SRDF
          if (frame_id1 == frame_id2)
          {
            if (verbose)
              std::cout << "Cannot disable collision between " << link1 << " and " << link2 << std::endl;
            continue;
          }
          
          typedef std::vector<CollisionPair> CollisionPairs_t;
          bool didRemove = false;
          for(CollisionPairs_t::iterator _colPair = geomModel.collisionPairs.begin();
              _colPair != geomModel.collisionPairs.end(); ) {
            const CollisionPair& colPair (*_colPair);
            bool remove =
              (
                  (geomModel.geometryObjects[colPair.first ].parentFrame == frame_id1)
               && (geomModel.geometryObjects[colPair.second].parentFrame == frame_id2)
              ) || (
                   (geomModel.geometryObjects[colPair.second].parentFrame == frame_id1)
                && (geomModel.geometryObjects[colPair.first ].parentFrame == frame_id2)
              );

            if (remove) {
              _colPair = geomModel.collisionPairs.erase(_colPair);
              didRemove = true;
            } else {
              ++_colPair;
            }
          }
          if(didRemove && verbose)
            std::cout << "Remove collision pair (" << link1 << "," << link2 << ")" << std::endl;
          
        }
      } // BOOST_FOREACH
    }
    
#endif // ifdef WITH_HPP_FCL
    

    ///
    /// \brief Get the neutral configuration of a given model associated to a SRDF file.
    ///        It throws if the SRDF file is incorrect.
    /// 
    /// \ingroup parsers_group
    /// 
    /// \param[in] model The Model for which we want the neutral config
    /// \param[in] filename The complete path to the SRDF file.
    /// \param[in] verbose Verbosity mode.
    ///
    /// \return The neutral configuration as an eigen vector
    inline Eigen::VectorXd getNeutralConfigurationFromSrdf(Model & model,
                                                           const std::string & filename,
                                                           const bool verbose) throw (std::invalid_argument)
    {
      const Eigen::VectorXd neutralConfig(model.nq);

      // Check extension
      const std::string extension = filename.substr(filename.find_last_of('.')+1);
      if (extension != "srdf")
      {
        const std::string exception_message (filename + " does not have the right extension.");
        throw std::invalid_argument(exception_message);
      }
      
      // Open file
      std::ifstream srdf_stream(filename.c_str());
      if (! srdf_stream.is_open())
      {
        const std::string exception_message (filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
      
      // Read xml stream
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(srdf_stream, pt);
      
      // Iterate over all tags directly children of robot
      BOOST_FOREACH(const ptree::value_type & v, pt.get_child("robot"))
      {
        // if we encounter a tag group_state
        if (v.first == "group_state")
        {
          const std::string name = v.second.get<std::string>("<xmlattr>.name");
          std::cout << name << std::endl;
          // Ensure that it is the half_sitting tag
          if( name == "half_sitting")
          {
            // Iterate over all the joint tags
            BOOST_FOREACH(const ptree::value_type & joint, v.second)
            {
              if (joint.first == "joint")
              {
                std::string joint_name = joint.second.get<std::string>("<xmlattr>.name");
                double joint_config = joint.second.get<double>("<xmlattr>.value");
                if (verbose)
                {
                  std::cout << "(" << joint_name << " , " << joint_config << ")" << std::endl;
                }
                // Search in model the joint and its config id
                Model::JointIndex joint_id = model.getJointId(joint_name);
                const JointModel & joint = model.joints[joint_id];

                if (joint_id != model.joints.size()) // != model.njoints
                {
                  model.neutralConfiguration(joint.idx_q()) = joint_config; // joint with 1 dof
                  // model.neutralConfiguration.segment(joint.idx_q(),joint.nq()) = joint_config; // joint with more than 1 dof
                }
                else
                {
                  if (verbose) std::cout << "The Joint " << joint_name << " was not found in model" << std::endl;
                }
              }
            }
            return neutralConfig;
          }

          
        }
      } // BOOST_FOREACH
      assert(false && "no half_sitting configuration found in the srdf file"); // Should we throw something here ?  
      return neutralConfig; // warning : uninitialized vector is returned
    }
  }
} // namespace se3

#endif // ifndef __se3_parser_srdf_hpp__
