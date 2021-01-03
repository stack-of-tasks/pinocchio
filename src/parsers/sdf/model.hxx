//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_multibody_parsers_sdf_model_hxx__
#define __pinocchio_multibody_parsers_sdf_model_hxx__

#include "pinocchio/math/matrix.hpp"
#include "pinocchio/parsers/sdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/contact-info.hpp"


#include <sdf/sdf.hh>
#include <ignition/math.hh>
#include <sstream>
#include <boost/foreach.hpp>
#include <limits>
#include <iostream>

namespace pinocchio
{
  namespace sdf
  {
    namespace details
    {

      static SE3 convertFromPose3d(const ignition::math::Pose3d& posePlacement)
      {
        const ignition::math::Quaterniond& q = posePlacement.Rot();
        const ignition::math::Vector3d& p = posePlacement.Pos();
        return SE3(SE3::Quaternion(q.W(),q.X(),q.Y(),q.Z()).matrix(),
                   SE3::Vector3(p.X(),p.Y(),p.Z()));
      }
      
      ///
      /// \brief Convert SDF Inertial quantity to Spatial Inertia.
      ///
      /// \param[in] Y The input URDF Inertia.
      ///
      /// \return The converted Spatial Inertia pinocchio::Inertia.
      ///
      static Inertia convertInertiaFromSdf(const ::sdf::ElementPtr inertial)
      {
        
        const ignition::math::Pose3d& pose =
          inertial->template Get<ignition::math::Pose3d>("pose");
        const double& mass = inertial->template Get<double>("mass");

        const ::sdf::ElementPtr inertiaElem = inertial->GetElement("inertia");
        const double ixx = inertiaElem->template Get<double>("ixx");
        const double ixy = inertiaElem->template Get<double>("ixy");
        const double ixz = inertiaElem->template Get<double>("ixz");
        const double iyy = inertiaElem->template Get<double>("iyy");
        const double iyz = inertiaElem->template Get<double>("iyz");
        const double izz = inertiaElem->template Get<double>("izz");

        const Inertia::Vector3 com(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        const Inertia::Matrix3 & R =
          Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),
                             pose.Rot().Y(),pose.Rot().Z()).matrix();

        Inertia::Matrix3 I;
        I << ixx,ixy,ixz,
             ixy,iyy,iyz,
             ixz,iyz,izz;
        
        return Inertia(mass,com,R*I*R.transpose());
      }

      
      ///
      /// \brief Recursive procedure for reading the SDF tree.
      ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
      ///
      /// \param[in] link The current SDF link.
      /// \param[in] model The model where the link must be added.
      ///
      void parseTree(const ::sdf::ElementPtr jointElement,
                     const std::map<std::string, ::sdf::ElementPtr> mapOfJoints,
                     const std::map<std::string, ::sdf::ElementPtr> mapOfLinks,
                     const std::map<std::string, std::vector<std::string> > childrenOfLinks,
                     ::pinocchio::urdf::details::UrdfVisitorBase & model,
                     PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) & contact_models)
      {
        typedef ::pinocchio::urdf::details::UrdfVisitorBase UrdfVisitorBase;
        typedef UrdfVisitorBase::Scalar Scalar;
        typedef UrdfVisitorBase::SE3 SE3;
        typedef UrdfVisitorBase::Vector Vector;
        typedef UrdfVisitorBase::Vector3 Vector3;
        typedef Model::FrameIndex FrameIndex;

        const std::string& jointName = jointElement->template Get<std::string>("name");

        std::string parentName;
        ignition::math::Pose3d parentPlacement;
        ::sdf::ElementPtr parentElement;
        parentName = jointElement->GetElement("parent")->Get<std::string>();
        parentElement = mapOfLinks.find(parentName)->second;
        parentPlacement =
          parentElement->template Get<ignition::math::Pose3d>("pose");
        
        const std::string childName =
          jointElement->GetElement("child")->Get<std::string>();
       
        std::cout << "Joint " << jointName << " connects " << parentName
                  << " link to " << childName << " link" << " with joint type "
                  << jointElement->template Get<std::string>("type")<<std::endl;

        const ::sdf::ElementPtr childElement = mapOfLinks.find(childName)->second;


        if (model.existFrame(childName, BODY)) {
          
          std::cout << "Link " << childName << " already exists with parent Joint id: " << model.getParentId(childName) <<std::endl;
          
          JointIndex parentJointId = model.getParentId(parentName);
          JointIndex childJointId = model.getParentId(childName);
          contact_models.push_back(::pinocchio::RigidContactModel(::pinocchio::CONTACT_3D,
                                                                  parentJointId,
                                                                  childJointId));
        }
        else {
        
            const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
            const Inertia Y = convertInertiaFromSdf(inertialElem);

            const ignition::math::Pose3d& childPlacement =
              childElement->template Get<ignition::math::Pose3d>("pose");

            const SE3 oMp = convertFromPose3d(parentPlacement);
            const SE3 oMc = convertFromPose3d(childPlacement);

            const SE3 jointPlacement = oMp.inverse() * oMc;

            std::ostringstream joint_info;

            FrameIndex parentFrameId = model.getBodyId(parentName);
            Vector max_effort(1), max_velocity(1), min_config(1), max_config(1);
            Vector spring_stiffness(1), spring_reference(1);
            Vector friction(Vector::Constant(1,0.)), damping(Vector::Constant(1,0.));
            ignition::math::Vector3d axis_ignition;
            Vector3 axis;

            const Scalar infty = std::numeric_limits<Scalar>::infinity();

            if (jointElement->HasElement("axis"))
            {
              const ::sdf::ElementPtr axisElem = jointElement->GetElement("axis");

              axis_ignition =
                axisElem->Get<ignition::math::Vector3d>("xyz");
              axis << axis_ignition.X(), axis_ignition.Y(), axis_ignition.Z();

              if (axisElem->HasElement("limit"))
              {
                const ::sdf::ElementPtr limitElem = axisElem->GetElement("limit");
                if (limitElem->HasElement("upper"))
                {
                  max_config[0] = limitElem->Get<double>("upper");
                }
                if (limitElem->HasElement("lower"))
                {
                  min_config[0] = limitElem->Get<double>("lower");
                }
                if (limitElem->HasElement("effort"))
                {
                  max_effort[0] = limitElem->Get<double>("effort");
                }
                if (limitElem->HasElement("velocity"))
                {
                  max_velocity[0] = limitElem->Get<double>("velocity");
                }
              }
              if (axisElem->HasElement("dynamics"))
              {
                const ::sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");
                if (dynamicsElem->HasElement("spring_reference"))
                {
                  spring_reference[0] = dynamicsElem->Get<double>("spring_reference");
                }
                if (dynamicsElem->HasElement("spring_stiffness"))
                {
                  spring_stiffness[0] = dynamicsElem->Get<double>("spring_stiffness");
                }
                if (dynamicsElem->HasElement("damping"))
                {
                  damping[0] = dynamicsElem->Get<double>("damping");
                }
              }
            }

            if (jointElement->template Get<std::string>("type") == "universal")
            {
            }
            else if (jointElement->template Get<std::string>("type") == "revolute")
            {
              joint_info << "joint REVOLUTE with axis";
              model.addJointAndBody(UrdfVisitorBase::REVOLUTE, axis,
                                    parentFrameId, jointPlacement, jointName,
                                    Y, childName,
                                    max_effort, max_velocity, min_config, max_config,
                                    friction,damping);
            }
            else if (jointElement->template Get<std::string>("type") == "gearbox")
            {
              joint_info << "joint GEARBOX with axis";
              //std::cerr<<"TODO: Fix Gearbox transmission"<<std::endl;
              model.addFixedJointAndBody(parentFrameId, jointPlacement, jointName,
                                         Y, childName);
            }
            else if (jointElement->template Get<std::string>("type") == "ball")
            {

              max_effort   = Vector::Constant(3, infty);
              max_velocity = Vector::Constant(3, infty);
              min_config   = Vector::Constant(4,-infty);
              max_config   = Vector::Constant(4, infty);
              min_config.setConstant(-1.01);
              max_config.setConstant( 1.01);
              friction = Vector::Constant(3, 0.);
              damping = Vector::Constant(3, 0.);

              joint_info << "joint BALL";
              //std::cerr<<"TODO: Fix BALL JOINT"<<std::endl;
              model.addJointAndBody(UrdfVisitorBase::SPHERICAL, axis,
                                    parentFrameId, jointPlacement, jointName,
                                    Y, childName,
                                    max_effort, max_velocity, min_config, max_config,
                                    friction,damping);
            }
            else
            {
              std::cerr<<"This type is yet to be implemented "<<jointElement->template Get<std::string>("type")<<std::endl;

            }


            const std::vector<std::string>& childrenOfLink =
              childrenOfLinks.find(childName)->second;

            for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
                childOfChild != std::end(childrenOfLink); ++childOfChild)
            {
              const ::sdf::ElementPtr childOfChildElement =
                mapOfJoints.find(*childOfChild)->second;
              parseTree(childOfChildElement, mapOfJoints, mapOfLinks, childrenOfLinks,
                        model, contact_models);
            }
        }
      }

      
      void parseRootTree(const std::string & filename,
                         ::pinocchio::urdf::details::UrdfVisitorBase& model,
                         PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)&
                         contact_models)
      {

        // load and check sdf file
        ::sdf::SDFPtr sdfElement(new ::sdf::SDF());
        ::sdf::init(sdfElement);
        if (!::sdf::readFile(filename, sdfElement))
        {
          throw std::invalid_argument("The file " + filename + " does not "
                                      "contain a valid SDF model 1.");
        }

        // start parsing model
        const ::sdf::ElementPtr rootElement = sdfElement->Root();

        if (!rootElement->HasElement("model"))
        {
          throw std::invalid_argument("The file " + filename + " does not "
                                      "contain a valid SDF model 2.");
        }

        const ::sdf::ElementPtr modelElement = rootElement->GetElement("model");

        const std::string& modelName = modelElement->template Get<std::string>("name");
        model.setName(modelName);

        std::cout << "Found " << modelName << " model!" << std::endl;

        std::map<std::string, ::sdf::ElementPtr> mapOfLinks, mapOfJoints;
        std::map<std::string, std::vector<std::string> > childrenOfLinks;

        // parse model links
        ::sdf::ElementPtr linkElement = modelElement->GetElement("link");
        while (linkElement)
        {
          const std::string linkName = linkElement->Get<std::string>("name");
          std::cout << "Found " << linkName << " link in "
                    << modelName << " model!" << std::endl;
          //Inserting data in std::map
          mapOfLinks.insert(std::make_pair(linkName, linkElement));
          childrenOfLinks.insert(std::make_pair(linkName, std::vector<std::string>()));
          linkElement = linkElement->GetNextElement("link");
        }

        childrenOfLinks.insert(std::make_pair("world", std::vector<std::string>()));
        
        // parse model joints
        ::sdf::ElementPtr jointElement = modelElement->GetElement("joint");
        while (jointElement)
        {
          const std::string jointName = jointElement->template Get<std::string>("name");
          std::string parentLinkName =
            jointElement->GetElement("parent")->template Get<std::string>();
          //Inserting data in std::map
          mapOfJoints.insert(std::make_pair(jointName, jointElement));
          //Create data of children of links
          childrenOfLinks.find(parentLinkName)->second.push_back(jointName);
          jointElement = jointElement->GetNextElement("joint");
        }

        //First joint connecting universe
        jointElement = mapOfJoints.find("static")->second;
        const std::string childName =
          jointElement->GetElement("child")->Get<std::string>();;
        const ::sdf::ElementPtr childElement = mapOfLinks.find(childName)->second;
        const ::sdf::ElementPtr inertialElem = childElement->GetElement("inertial");
        const Inertia Y = convertInertiaFromSdf(inertialElem);

        std::cerr<<"Adding rootjoint:"<<std::endl;
        model.addRootJoint(convertInertiaFromSdf(inertialElem), childName);
        std::cerr<<"Added rootjoint:"<<std::endl;
        const std::vector<std::string>& childrenOfLink =
          childrenOfLinks.find(childName)->second;
        for(std::vector<std::string>::const_iterator childOfChild = std::begin(childrenOfLink);
            childOfChild != std::end(childrenOfLink); ++childOfChild)
        {
          parseTree(mapOfJoints.find(*childOfChild)->second, mapOfJoints, mapOfLinks, childrenOfLinks, model, contact_models);
        }
      }
    }
    
    template<typename Scalar, int Options,
             template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)& contact_models,
               const bool verbose)
    {
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor (model);

      if (verbose) visitor.log = &std::cout;
      details::parseRootTree(filename, visitor, contact_models);
      return model;
    }
  }
}

#endif // ifndef __pinocchio_parsers_sdf_hpp__
