//
// Copyright (c) 2017-2018 CNRS
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

#ifndef __se3_parser_srdf_hxx__
#define __se3_parser_srdf_hxx__

#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include <iostream>

// Read XML file with boost
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>

namespace se3
{
  namespace srdf
  {
    
#ifdef WITH_HPP_FCL
    namespace details
    {
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
      void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                GeometryModel & geomModel,
                                std::istream & stream,
                                const bool verbose = false)
      {
        // Read xml stream
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(stream, pt);

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

            const typename Model::FrameIndex frame_id1 = model.getBodyId(link1);
            const typename Model::FrameIndex frame_id2 = model.getBodyId(link2);

            // Malformed SRDF
            if (frame_id1 == frame_id2)
            {
              if (verbose)
                std::cout << "Cannot disable collision between " << link1 << " and " << link2 << std::endl;
              continue;
            }

            typedef GeometryModel::CollisionPairVector CollisionPairVector;
            bool didRemove = false;
            for(CollisionPairVector::iterator _colPair = geomModel.collisionPairs.begin();
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
    } // namespace details

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairs(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
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

      details::removeCollisionPairs(model, geomModel, srdf_stream, verbose);
    }

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void removeCollisionPairsFromXML(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                     GeometryModel & geomModel,
                                     const std::string & xmlString,
                                     const bool verbose)
    {
      std::istringstream srdf_stream(xmlString);
      details::removeCollisionPairs(model, geomModel, srdf_stream, verbose);
    }
    
#endif // ifdef WITH_HPP_FCL
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bool loadRotorParameters(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                             const std::string & filename,
                             const bool verbose) throw (std::invalid_argument)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::JointModel JointModel;
      
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
        // if we encounter a tag rotor_params
        if (v.first == "rotor_params")
        {
          // Iterate over all the joint tags
          BOOST_FOREACH(const ptree::value_type & joint, v.second)
          {
            if (joint.first == "joint")
            {
              std::string joint_name = joint.second.get<std::string>("<xmlattr>.name");
              const Scalar rotor_mass = (Scalar)joint.second.get<double>("<xmlattr>.mass");
              const Scalar rotor_gr = (Scalar)joint.second.get<double>("<xmlattr>.gear_ratio");
              if (verbose)
              {
                std::cout << "(" << joint_name << " , " <<
                rotor_mass << " , " << rotor_gr << ")" << std::endl;
              }
              // Search in model the joint and its config id
              typename Model::JointIndex joint_id = model.getJointId(joint_name);

              if (joint_id != model.joints.size()) // != model.njoints
              {
                const JointModel & joint = model.joints[joint_id];
                assert(joint.nv()==1);
                model.rotorInertia(joint.idx_v()) = rotor_mass;
                model.rotorGearRatio(joint.idx_v()) = rotor_gr;  // joint with 1 dof
              }
              else
              {
                if (verbose) std::cout << "The Joint " << joint_name << " was not found in model" << std::endl;
              }
            }
          }
          return true; 
        }
      }
      assert(false && "no rotor params found in the srdf file");  
      return false; // warning : uninitialized vector is returned
    }
    
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    typename ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType
    getNeutralConfiguration(ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                            const std::string & filename,
                            const bool verbose) throw (std::invalid_argument)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef typename Model::JointModel JointModel;
      
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
          // Ensure that it is the half_sitting tag
          if( name == "half_sitting")
          {
            // Iterate over all the joint tags
            BOOST_FOREACH(const ptree::value_type & joint, v.second)
            {
              if (joint.first == "joint")
              {
                std::string joint_name = joint.second.get<std::string>("<xmlattr>.name");
                const Scalar joint_config = (Scalar)joint.second.get<double>("<xmlattr>.value");
                if (verbose)
                {
                  std::cout << "(" << joint_name << " , " << joint_config << ")" << std::endl;
                }
                // Search in model the joint and its config id
                typename Model::JointIndex joint_id = model.getJointId(joint_name);

                if (joint_id != model.joints.size()) // != model.njoints
                {
                  const JointModel & joint = model.joints[joint_id];
                  assert(joint.nq() == 1 && "SRDF:half_sitting only handles 1 DoF joints");
                  model.neutralConfiguration(joint.idx_q()) = joint_config; // joint with 1 dof
                  // model.neutralConfiguration.segment(joint.idx_q(),joint.nq()) = joint_config; // joint with more than 1 dof
                }
                else
                {
                  if (verbose) std::cout << "The Joint " << joint_name << " was not found in model" << std::endl;
                }
              }
            }
            return model.neutralConfiguration;
          }
          
        }
      } // BOOST_FOREACH
      
      assert(false && "no half_sitting configuration found in the srdf file"); // Should we throw something here ?  
      return ModelTpl<Scalar,Options,JointCollectionTpl>::ConfigVectorType::Constant(model.nq,(Scalar)NAN); // warning : uninitialized vector is returned
    }
  }
} // namespace se3

#endif // ifndef __se3_parser_srdf_hxx__
