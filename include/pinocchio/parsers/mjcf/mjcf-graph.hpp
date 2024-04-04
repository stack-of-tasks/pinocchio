//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_parsers_mjcf_graph_hpp__
#define __pinocchio_parsers_mjcf_graph_hpp__

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/joint/joints.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include <sstream>
#include <limits>
#include <iostream>

namespace pinocchio
{
    namespace mjcf
    {
        namespace details 
        {
            struct MjcfGraph;
            struct MjcfJoint;

            /// @brief Informations that are stocked in the XML tag compile.
            /// 
            struct MjcfCompiler
            {
                public:
                    // Global attribute to use limit that are in the model or not
                    bool autolimits = true;
                    // Attribute to keep or not the full path of files specified in the model
                    bool strippath = false;
                    // Value for angle conversion (Mujoco default - degrees)
                    double angle_converter = boost::math::constants::pi<double>() / 180.0;
                    // Euler Axis to use to convert angles representation to quaternion
                    Eigen::Matrix3d mapEulerAngles;
                    
                    /// @brief Convert the angle in radian if model was declared to use degree
                    /// @param angle_ angle to convert
                    /// @return converted angle
                    double convertAngle(const double &angle_) const;

                    /// @brief Convert the euler angles according to the convention declared in the compile tag. 
                    /// @param angles Euler angles
                    /// @return Quaternion representation of the euler angles
                    Eigen::Matrix3d convertEuler(const Eigen::Vector3d &angles) const;
            };

            /// @brief Structure to stock all default classes information
            struct MjcfClass
            {
                public:
                    typedef boost::property_tree::ptree ptree;

                    // name of the default class
                    std::string className;
                    // Ptree associated with the class name
                    ptree classElement;
            };

            /// @brief All Bodies informations extracted from mjcf model
            struct MjcfBody
            {
                public:
                    // Name of the body
                    std::string bodyName;
                    // Name of the parent 
                    std::string bodyParent = "";
                    // Name of the default class used by this body (optional)
                    std::string bodyClassName;
                    // Special default class, that is common to all bodies and children if not specified otherwise
                    std::string childClass = "";

                    // Position of the body wrt to the previous body
                    SE3 bodyPlacement = SE3::Identity();
                    // Body inertia
                    Inertia bodyInertia = Inertia::Identity();

                    // Vector of joints associated with the body
                    std::vector<MjcfJoint> jointChildren;
            };

            /// @brief All joint limits
            struct RangeJoint
            {
                // Max effort
                Eigen::VectorXd maxEffort;
                // Max velocity
                Eigen::VectorXd maxVel;
                // Max position
                Eigen::VectorXd maxConfig;
                // Min position
                Eigen::VectorXd minConfig;

                // Join Stiffness
                Eigen::VectorXd springStiffness;
                //  joint position or angle in which the joint spring (if any) achieves equilibrium
                Eigen::VectorXd springReference;

                // friction applied in this joint
                Eigen::VectorXd friction;
                // Damping applied by this joint. 
                Eigen::VectorXd damping;

                // Armature inertia created by this joint
                double armature = 0.;
                // Dry friction.
                double frictionLoss = 0.;

                RangeJoint() = default;
                explicit RangeJoint(double v)
                {
                    const double infty = std::numeric_limits<double>::infinity();
                    maxVel = Eigen::VectorXd::Constant(1, infty);
                    maxEffort = Eigen::VectorXd::Constant(1, infty);
                    minConfig = Eigen::VectorXd::Constant(1, - infty);
                    maxConfig = Eigen::VectorXd::Constant(1, infty);
                    springStiffness = Eigen::VectorXd::Constant(1, v);
                    springReference = Eigen::VectorXd::Constant(1, v);;
                    friction = Eigen::VectorXd::Constant(1,0.);
                    damping = Eigen::VectorXd::Constant(1,0.);  
                }

                /// @brief Set dimension to the limits to match the joint nq and nv.
                /// @tparam Nq joint configuration
                /// @tparam Nv joint velocity
                /// @return Range with new dimension
                template<int Nq, int Nv>
                RangeJoint setDimension() const;

                /// @brief Concatenate 2 rangeJoint
                /// @tparam Nq old_range, joint configuration 
                /// @tparam Nv old_range, joint velocity
                /// @param range to concatenate with
                /// @return Concatenated range.
                template<int Nq, int Nv>
                RangeJoint concatenate(const RangeJoint &range) const;
            };

            /// @brief All joint information parsed from the mjcf model
            struct MjcfJoint
            {
                public:
                    typedef boost::property_tree::ptree ptree;

                    // Name of the joint
                    std::string jointName = "free";
                    // Placement of the joint wrt to its body - default Identity
                    SE3 jointPlacement = SE3::Identity();

                    // axis of the joint - default "0 0 1"
                    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
                    // Limits that applie to this joint
                    RangeJoint range{1};

                    // type of the joint (hinge, ball, slide, free) - default "hinge"
                    std::string jointType = "hinge";

                    /// @param el ptree joint node 
                    /// @param currentBody body to which the joint belongs to 
                    /// @param currentGraph current Mjcf graph (needed to get compiler information)
                    void fill(const ptree &el, const MjcfBody &currentBody, const MjcfGraph &currentGraph);
                    
                    /// @brief Go through a joint node (default class or not) and parse info into the structure
                    /// @param el ptree joint node
                    /// @param use_limits whether to parse the limits or not 
                    void goThroughElement(const ptree &el, bool use_limits);
            };

            /// @brief The graph which contains all information taken from the mjcf file
            struct MjcfGraph
            {
                public:
                    typedef boost::property_tree::ptree ptree;
                    typedef std::vector<std::string> VectorOfStrings;
                    typedef std::unordered_map<std::string, MjcfBody> BodyMap_t;
                    typedef std::unordered_map<std::string, MjcfClass> ClassMap_t;

                    // Compiler Info needed to properly parse the rest of file
                    MjcfCompiler compilerInfo;
                    // Map of default classes 
                    ClassMap_t mapOfClasses;
                    // Map of bodies
                    BodyMap_t mapOfBodies;

                    // property tree where xml file is stored
                    ptree pt;

                    // Ordered list of bodies
                    VectorOfStrings bodiesList;

                    // Name of the model
                    std::string modelName;

                    // Urdf Visitor to add joint and body
                    typedef pinocchio::urdf::details::UrdfVisitor<double, 0, ::pinocchio::JointCollectionDefaultTpl > UrdfVisitor;
                    UrdfVisitor& urdfVisitor;

                    /// @brief graph constructor
                    /// @param urdfVisitor 
                    MjcfGraph(UrdfVisitor& urdfVisitor):
                    urdfVisitor(urdfVisitor)
                    {}

                    /// @brief Convert pose of an mjcf element into SE3
                    /// @param el ptree element with all the pose element
                    /// @return pose in SE3
                    SE3 convertPosition(const ptree &el) const;

                    /// @brief Convert Inertia of an mjcf element into Inertia model of pinocchio
                    /// @param el ptree element with all the inertial information
                    /// @return Inertia element in pinocchio
                    Inertia convertInertiaFromMjcf(const ptree &el) const;

                    /// @brief Go through the default part of the file and get all the class name. Fill the mapOfDefault for later use.
                    /// @param el ptree element. Root of the default
                    void parseDefault(ptree &el, const ptree &parent);

                    /// @brief Go through the main body of the mjcf file "worldbody" to get all the info ready to create the model.
                    /// @param el root of the tree
                    /// @param parentName name of the parentBody in the robot tree
                    void parseJointAndBody(const ptree &el, const boost::optional<std::string> &childClass, const std::string &parentName="");

                    /// @brief Parse all the info from the compile node into compilerInfo
                    /// @param el ptree compile node
                    void parseCompiler(const ptree &el);

                    /// @brief parse the mjcf file into a graph
                    void parseGraph();

                    /// @brief parse the mjcf file into a graph
                    /// @param xmlStr xml file name 
                    void parseGraphFromXML(const std::string &xmlStr);

                    /// @brief Create a joint to add to the joint composite if needed
                    /// @tparam TypeX joint with axis X
                    /// @tparam TypeY joint with axis Y
                    /// @tparam TypeZ joint with axis Z
                    /// @tparam TypeUnaligned joint with axis unaligned
                    /// @param axis axis of the joint
                    /// @return one of the joint with the right axis
                    template <typename TypeX, typename TypeY, typename TypeZ, typename TypeUnaligned>
                    JointModel createJoint(const Eigen::Vector3d& axis);

                    /// @brief Add a joint to the model. only needed when a body has a solo joint child
                    /// @param jointInfo The joint to add to the tree
                    /// @param currentBody The body associated with the joint
                    void addSoloJoint(const MjcfJoint &jointInfo, const MjcfBody &currentBody);

                    /// @brief Use all the infos that were parsed from the xml file to add a body and joint to the model
                    /// @param nameOfBody Name of the body to add
                    void fillModel(const std::string &nameOfBody);

                    /// @brief Fill the pinocchio model with all the infos from the graph
                    void parseRootTree();      
            };

        } // details
    } //mjcf
} //pinocchio

#endif // __pinocchio__parsers_mjcf_graph_hpp__
