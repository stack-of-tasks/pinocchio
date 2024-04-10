//
// Copyright (c) 2016-2024 CNRS INRIA
//

#include "pinocchio/parsers/mjcf/mjcf-graph.hpp"

namespace pinocchio
{
    namespace mjcf
    {
        namespace details
        {
            typedef boost::property_tree::ptree ptree;
            typedef pinocchio::urdf::details::UrdfVisitor<double, 0, ::pinocchio::JointCollectionDefaultTpl > UrdfVisitor;

            //supported elements from mjcf 
            static const std::array<std::string, 3> ELEMENTS = {"joint", "geom", "site"};

            static std::istringstream getConfiguredStringStream(const std::string& str) 
            {
                std::istringstream posStream(str);
                posStream.exceptions(std::ios::failbit);
                return posStream;
            }
            
            template<int N>
            static Eigen::Vector<double, N> getVectorFromStream(const std::string &str)
            {
                std::istringstream stream = getConfiguredStringStream(str);
                Eigen::Vector<double, N> vector;
                for(int i = 0; i < N; i++)
                    stream >> vector(i);
                
                return vector;
            }

            /// @brief Copy the value of ptree src into dst
            /// @param src ptree to copy
            /// @param dst ptree where copy is made
            static void copyPtree(const ptree& src, ptree& dst)
            {
                for(const ptree::value_type& v: src)
                    dst.put(ptree::path_type(v.first), v.second.data());
            }
            
            /// @brief Update class Element in order to have all info of parent classes
            /// @param current current class
            /// @param dst parent class
            static void updateClassElement(ptree& current, const ptree& parent)
            {
                for(const std::string& el: ELEMENTS)
                {
                    std::string path = el + ".<xmlattr>";
                    if(parent.get_child_optional(path))
                    {
                        ptree attr_parent = parent.get_child(path, ptree());
                        ptree attr_current = current.get_child(path, ptree());
                        // To only copy non existing attribute in current, we copy all current
                        // attribute (replacing) into a parent copy then we replace current with the new
                        // ptree
                        copyPtree(attr_current, attr_parent);
                        current.put_child(path, attr_parent);
                    }
                }
            }

            double MjcfCompiler::convertAngle(const double &angle_) const
            {
                return  angle_ * angle_converter;
            }
            
            Eigen::Matrix3d MjcfCompiler::convertEuler(const Eigen::Vector3d &angles) const
            {
                convertAngle(angles(0));
                Eigen::Matrix3d aa1 = Eigen::AngleAxisd(angles(0), mapEulerAngles.col(0)).toRotationMatrix();
                convertAngle(angles(1));
                Eigen::Matrix3d aa2 = Eigen::AngleAxisd(angles(1), mapEulerAngles.col(1)).toRotationMatrix();
                convertAngle(angles(2));
                Eigen::Matrix3d aa3 = Eigen::AngleAxisd(angles(2), mapEulerAngles.col(2)).toRotationMatrix();

                return aa1*aa2*aa3;
            }

            template<int Nq, int Nv>
            RangeJoint RangeJoint::setDimension() const
            {
                typedef UrdfVisitor::Vector Vector;
                const double infty = std::numeric_limits<double>::infinity();

                RangeJoint ret;
                ret.maxEffort   = Vector::Constant(Nv, infty);
                ret.maxVel = Vector::Constant(Nv, infty);
                ret.maxConfig   = Vector::Constant(Nq, 1.01);
                ret.minConfig   = Vector::Constant(Nq,-1.01);
                ret.friction = Vector::Constant(Nv, 0.);
                ret.damping = Vector::Constant(Nv, 0.);
                ret.armature = armature;
                ret.frictionLoss = frictionLoss;
                ret.springStiffness = springStiffness;
                ret.springReference = springReference;
                return ret;
            }

            template<int Nq, int Nv>
            RangeJoint RangeJoint::concatenate(const RangeJoint& range) const
            {
                assert(range.maxEffort.size() == Nv);
                assert(range.minConfig.size() == Nq);

                RangeJoint ret(*this);
                ret.maxEffort.conservativeResize(maxEffort.size()+Nv);
                ret.maxEffort.tail(Nv) = range.maxEffort;
                ret.maxVel.conservativeResize(maxVel.size()+Nv);
                ret.maxVel.tail(Nv) = range.maxVel;

                ret.minConfig.conservativeResize(minConfig.size()+Nq);
                ret.minConfig.tail(Nq) = range.minConfig;
                ret.maxConfig.conservativeResize(maxConfig.size()+Nq);
                ret.maxConfig.tail(Nq) = range.maxConfig;

                ret.damping.conservativeResize(damping.size()+Nv);
                ret.damping.tail(Nv) =range.damping;
                ret.friction.conservativeResize(friction.size()+Nv);
                ret.friction.tail(Nv) =range.friction;

                ret.springReference.conservativeResize(springReference.size()+1);
                ret.springReference.tail(1) =range.springReference;
                ret.springStiffness.conservativeResize(springStiffness.size()+1);
                ret.springStiffness.tail(1) =range.springStiffness;
                return ret;
            }

            void MjcfJoint::goThroughElement(const ptree &el, bool use_limits)
            {
        
                if(!use_limits && el.get_optional<std::string>("<xmlattr>.range"))
                    throw std::invalid_argument("Range limit is specified but it was not specify to use it.");

                //Type 
                auto type_s = el.get_optional<std::string>("<xmlattr>.type");
                if(type_s)
                    jointType = *type_s;

                //Axis
                auto ax = el.get_optional<std::string>("<xmlattr>.axis");
                
                if(ax) 
                    axis = getVectorFromStream<3>(*ax);
                
                // Range
                auto range_ = el.get_optional<std::string>("<xmlattr>.range");
                if(range_)
                {
                    Eigen::Vector2d rangeT = getVectorFromStream<2>(*range_);
                    range.minConfig[0] = rangeT(0);
                    range.maxConfig[0] = rangeT(1);
                }
                // Effort limit
                range_ = el.get_optional<std::string>("<xmlattr>.actuatorfrcrange");
                if(range_)
                {
                    Eigen::Vector2d rangeT = getVectorFromStream<2>(*range_);
                    range.maxEffort[0] = rangeT(1);
                }
                
                // Spring
                auto value = el.get_optional<double>("<xmlattr>.springref");
                if(value)
                    range.springReference[0] = *value;

                // damping
                value = el.get_optional<double>("<xmlattr>.damping");
                if(value)
                    range.damping[0] = *value;

                value = el.get_optional<double>("<xmlattr>.armature");
                if(value)
                    range.armature = *value;
                
                // friction loss
                value = el.get_optional<double>("<xmlattr>.frictionloss");
                if(value)
                    range.frictionLoss = *value;
            }

            void MjcfJoint::fill(const ptree &el, const MjcfBody &currentBody, const MjcfGraph &currentGraph)
            {
                bool use_limit = true;
                // Name
                auto name_s = el.get_optional<std::string>("<xmlattr>.name");
                if(name_s)
                    jointName = *name_s;
                else
                    jointName = currentBody.bodyName + "Joint";

                // Check if we need to check for limited argument
                if(!currentGraph.compilerInfo.autolimits)
                {
                    use_limit = false;
                    auto use_ls = el.get_optional<std::string>("<xmlattr>.limited");
                    use_limit = *use_ls == "true";

                }
                
                // Placement
                jointPlacement = currentGraph.convertPosition(el);

                //ChildClass < Class < Real Joint
                // childClass
                if(currentBody.childClass != "")
                {
                    const MjcfClass& classE = currentGraph.mapOfClasses.at(currentBody.childClass);
                    if(auto joint_p = classE.classElement.get_child_optional("joint"))
                        goThroughElement(*joint_p, use_limit);
                }
                //Class 
                auto cl_s = el.get_optional<std::string>("<xmlattr>.class");
                if(cl_s)
                {
                    std::string className = *cl_s;
                    const MjcfClass&  classE = currentGraph.mapOfClasses.at(className);
                    if(auto joint_p = classE.classElement.get_child_optional("joint"))
                        goThroughElement(*joint_p, use_limit);
                }
                // Joint
                goThroughElement(el, use_limit);
            }

            SE3 MjcfGraph::convertPosition(const ptree &el) const
            {
                SE3 placement;
                placement.setIdentity();
                
                // position
                auto pos = el.get_optional<std::string>("<xmlattr>.pos");
                if(pos)
                    placement.translation() = getVectorFromStream<3>(*pos);
                
                /////////// Rotation 
                // Quaternion (w, x, y, z)
                auto rot_s = el.get_optional<std::string>("<xmlattr>.quat");
                if(rot_s)
                {
                    Eigen::Vector4d quat = getVectorFromStream<4>(*rot_s);
                    
                    Eigen::Quaterniond quaternion(quat(0), quat(1), quat(2), quat(3));
                    quaternion.normalize();
                    placement.rotation() =  quaternion.toRotationMatrix();
                }
                // Axis Angle
                else if (rot_s = el.get_optional<std::string>("<xmlattr>.axisangle"))
                {
                    Eigen::Vector4d axis_angle = getVectorFromStream<4>(*rot_s);

                    double angle = axis_angle(3);
                    compilerInfo.convertAngle(angle);

                    Eigen::AngleAxisd angleAxis(angle, axis_angle.head(3));
                    placement.rotation() = angleAxis.toRotationMatrix();
                }
                // Euler Angles
                else if (rot_s = el.get_optional<std::string>("<xmlattr>.euler"))
                {
                    Eigen::Vector3d angles = getVectorFromStream<3>(*rot_s);

                    placement.rotation() = compilerInfo.convertEuler(angles);                    
                }
                else if(rot_s = el.get_optional<std::string>("<xmlattr>.xyaxes"))
                {
                    Eigen::Vector<double, 6> xyaxes = getVectorFromStream<6>(*rot_s);

                    Eigen::Vector3d xAxis = xyaxes.head(3);
                    xAxis.normalize();
                    Eigen::Vector3d yAxis = xyaxes.tail(3); 

                    // make y axis orthogonal to x axis, normalize 
                    double d = xAxis.dot(yAxis);
                    yAxis -= xAxis*d; 
                    yAxis.normalize();
                    
                    Eigen::Vector3d zAxis = xAxis.cross(yAxis);
                    zAxis.normalize();

                    Eigen::Matrix3d rotation;
                    rotation.col(0) = xAxis;
                    rotation.col(1) = yAxis;
                    rotation.col(2) = zAxis;

                    placement.rotation() = rotation;
                }
                else if(rot_s = el.get_optional<std::string>("<xmlattr>.zaxis"))
                {
                    Eigen::Vector3d zaxis = getVectorFromStream<3>(*rot_s);
                    // Compute the rotation matrix that maps z_axis to unit z
                    placement.rotation() = Eigen::Quaterniond::FromTwoVectors(zaxis, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                }
                return placement;
            }

            Inertia MjcfGraph::convertInertiaFromMjcf(const ptree &el) const
            {
                const double mass = el.get<double>("<xmlattr>.mass");
                if(mass < 0)
                    throw std::invalid_argument("Mass of body is not supposed to be negative");

                Inertia::Vector3 com;
                auto com_s = el.get_optional<std::string>("<xmlattr>.pos");
                if(com_s)
                    com = getVectorFromStream<3>(*com_s);
                else
                    com = Inertia::Vector3::Zero();

            
                const Inertia::Matrix3 R = convertPosition(el).rotation();

                Inertia::Matrix3 I = Eigen::Matrix3d::Identity();

                auto inertia_s = el.get_optional<std::string>("<xmlattr>.diaginertia");
                if(inertia_s)
                {
                    Eigen::Vector3d inertiaDiag = getVectorFromStream<3>(*inertia_s);
                    I = inertiaDiag.asDiagonal();
                }

                else if (inertia_s = el.get_optional<std::string>("<xmlattr>.fullinertia"))
                {
                    // M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)
                    std::istringstream inertiaStream = getConfiguredStringStream(*inertia_s);
                    inertiaStream >> I(0, 0);
                    inertiaStream >> I(1, 1);
                    inertiaStream >> I(2, 2);
                    inertiaStream >> I(0, 1);
                    inertiaStream >> I(0, 2);
                    inertiaStream >> I(1, 2);

                    I(1, 0) = I(0, 1);
                    I(2, 0) = I(0, 2);
                    I(2, 1) = I(1, 2);
                }

                return Inertia(mass, com, R*I*R.transpose());
            }

            void MjcfGraph::parseJointAndBody(const ptree &el, const boost::optional<std::string> &childClass, const std::string &parentName)
            {
                MjcfBody currentBody;
                auto chcl_s = childClass;

                for(const ptree::value_type &v: el)
                {
                    // Current body node
                    if(v.first == "<xmlattr>")
                    {
                        // Name
                        auto name_s = v.second.get_optional<std::string>("name");
                        if(name_s)
                            currentBody.bodyName = *name_s;
                        else
                            currentBody.bodyName = parentName + "Bis";
                        
                        currentBody.bodyParent = parentName;
                        currentBody.bodyPlacement = convertPosition(el);

                        bodiesList.push_back(currentBody.bodyName);

                        if(auto chcl_st = v.second.get_optional<std::string>("childclass"))
                            chcl_s = chcl_st;

                        if(childClass)
                            currentBody.childClass = *chcl_s;
            
                        //Class 
                        auto cl_s = v.second.get_optional<std::string>("class");
                        if(cl_s)
                            currentBody.bodyClassName = *cl_s;

                        // Still need to deal with gravcomp and figure out if we need mocap, and user param...  
                    }
                    // Inertia
                    if(v.first == "inertial")
                        currentBody.bodyInertia = convertInertiaFromMjcf(v.second);
                    
                    // Joint
                    if(v.first == "joint")
                    {
                        MjcfJoint currentJoint;
                        currentJoint.fill(v.second, currentBody, *this);
                        currentBody.jointChildren.push_back(currentJoint);

                    }
                    if(v.first == "freejoint")
                    {
                        MjcfJoint currentJoint;
                        currentJoint.jointType = "free";
                        auto jointName = v.second.get_optional<std::string>("<xmlattr>.name");
                        if(jointName)
                            currentJoint.jointName = *jointName;
                        else
                            currentJoint.jointName = currentBody.bodyName + "_free";

                        currentBody.jointChildren.push_back(currentJoint);
                    }
                    if(v.first == "body")
                        parseJointAndBody(v.second, chcl_s, currentBody.bodyName);   
                }
                mapOfBodies.insert(std::make_pair(currentBody.bodyName, currentBody));
            }

            void MjcfGraph::parseDefault(ptree &el, const ptree &parent)
            {
                boost::optional<std::string> nameClass;
                for(ptree::value_type &v : el)
                {
                    if(v.first == "<xmlattr>")
                    {
                        nameClass = v.second.get_optional<std::string>("class");
                        if(nameClass)
                        {
                            MjcfClass classDefault;
                            classDefault.className = *nameClass;
                            updateClassElement(el, parent);
                            classDefault.classElement = el;
                            mapOfClasses.insert(std::make_pair(*nameClass, classDefault));
                        }
                        else
                            throw std::invalid_argument("Class does not have a name. Cannot parse model.");
                    }
                    if(v.first == "default")
                        parseDefault(v.second, el);
                }
            }

            void MjcfGraph::parseCompiler(const ptree &el)
            {
                // get autolimits
                auto auto_s = el.get_optional<std::string>("<xmlattr>.autolimits");
                if(auto_s)
                    if(*auto_s == "true")
                        compilerInfo.autolimits = true;

                // strip path
                auto strip_s = el.get_optional<std::string>("<xmlattr>.strippath");
                if(strip_s)
                    if(*strip_s == "true")
                        compilerInfo.strippath = true;

                // angle radian or degree
                auto angle_s = el.get_optional<std::string>("<xmlattr>.angle");
                if(angle_s)
                    if(*angle_s == "radian")
                        compilerInfo.angle_converter = 1;

                auto eulerS = el.get_optional<std::string>("<xmlattr>.eulerseq");
                if(eulerS)
                {
                    std::string eulerseq = *eulerS;
                    if(eulerseq.find_first_not_of("xyzXYZ") != std::string::npos || eulerseq.size() != 3)
                            throw std::invalid_argument("Model tried to use euler angles but euler sequence is wrong");
                    else
                    {
                        // get index combination
                        for(std::size_t i = 0; i < eulerseq.size(); i++)
                        {
                            switch(eulerseq.at(i))
                            {
                                case 'x':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitX();
                                    break;
                                case 'X':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitX();
                                    break;
                                case 'y':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitY();
                                    break;
                                case 'Y':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitY();
                                    break;
                                case 'z':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitZ();
                                    break;
                                case 'Z':
                                    compilerInfo.mapEulerAngles.col(i) = Eigen::Vector3d::UnitZ();
                                    break;
                                default:
                                    throw std::invalid_argument("Euler Axis does not exist");
                                    break;
                            }
                        }
                    }
                }
            }

            void MjcfGraph::parseGraph()
            {
                boost::property_tree::ptree el;
                if(pt.get_child_optional("mujoco"))
                    el = pt.get_child("mujoco");
                else
                    throw std::invalid_argument("This is not a standard mujoco model. Cannot parse it.");

                for(const ptree::value_type &v : el)
                {
                    // get model name
                    if(v.first == "<xmlattr>")
                    {
                        auto n_s = v.second.get_optional<std::string>("model");
                        if(n_s)
                            modelName = *n_s;
                        else
                            throw std::invalid_argument("Model is missing a name. Cannot parse it");
                    }
                    
                    if(v.first == "compiler")
                        parseCompiler(el.get_child("compiler"));

                    if(v.first == "default")
                        parseDefault(el.get_child("default"), el); 
                    
                    if(v.first == "option")
                        throw std::invalid_argument("Options are not supported yet");
                    if(v.first == "composite")
                        throw std::invalid_argument("Composite Bodies are not supported yet");
                    if(v.first == "worldbody")
                    {
                        boost::optional<std::string> childClass;
                        parseJointAndBody(el.get_child("worldbody").get_child("body"), childClass); 
                    }
                }
            }

            void MjcfGraph::parseGraphFromXML(const std::string &xmlStr)
            {
                boost::property_tree::read_xml(xmlStr, pt);
                parseGraph();
            }

            template <typename TypeX, typename TypeY, typename TypeZ, typename TypeUnaligned>
            JointModel MjcfGraph::createJoint(const Eigen::Vector3d& axis)
            {
                if( axis.isApprox(Eigen::Vector3d::UnitX()))
                    return TypeX();
                else if( axis.isApprox(Eigen::Vector3d::UnitY()))
                    return TypeY();
                else if( axis.isApprox(Eigen::Vector3d::UnitZ()))
                    return TypeZ();
                else
                    return TypeUnaligned(axis.normalized());
            }
            
            void MjcfGraph::addSoloJoint(const MjcfJoint &joint, const MjcfBody &currentBody)
            {
                const FrameIndex parentFrameId = urdfVisitor.getBodyId(currentBody.bodyParent);
                // get body pose in body parent
                const SE3 bodyPose = currentBody.bodyPlacement;
                Inertia inert = currentBody.bodyInertia;

                SE3 jointInParent = bodyPose * joint.jointPlacement;
                SE3 bodyInJoint = joint.jointPlacement.inverse();
                UrdfVisitor::JointType jType;

                RangeJoint range;

                if(joint.jointType == "free")
                {
                    urdfVisitor << "Free Joint "<< '\n';
                    range = joint.range.setDimension<7, 6>();
                    jType = UrdfVisitor::FLOATING;
                }
                else if(joint.jointType == "slide")
                {
                    urdfVisitor << "joint prismatic with axis " <<  joint.axis << '\n';
                    range = joint.range;
                    jType = UrdfVisitor::PRISMATIC; 
                }
                else if(joint.jointType == "ball")
                {
                    urdfVisitor << "Sphere Joint "  << '\n';
                    range = joint.range.setDimension<4, 3>();
                    jType = UrdfVisitor::SPHERICAL;

                }
                else if(joint.jointType == "hinge")
                {
                    urdfVisitor << "joint revolute with axis " <<  joint.axis << '\n';
                    range = joint.range;
                    jType = UrdfVisitor::REVOLUTE;
                }
                else
                    throw std::invalid_argument("Unknown joint type");
                
                urdfVisitor.addJointAndBody(jType,joint.axis,
                                            parentFrameId, jointInParent, joint.jointName,
                                            inert, bodyInJoint, currentBody.bodyName, range.maxEffort, range.maxVel,
                                            range.minConfig, range.maxConfig, range.friction, range.damping);

                // Add armature info
                JointIndex j_id = urdfVisitor.getJointId(joint.jointName);
                urdfVisitor.model.armature[j_id-1] = range.armature;
            }

            void MjcfGraph::fillModel(const std::string &nameOfBody)
            {
                typedef UrdfVisitor::SE3 SE3;

                MjcfBody currentBody = mapOfBodies.at(nameOfBody);
                // get parent body frame
                const FrameIndex parentFrameId = urdfVisitor.getBodyId(currentBody.bodyParent);

                // get body pose in body parent
                const SE3 bodyPose = currentBody.bodyPlacement;
                Inertia inert = currentBody.bodyInertia;

                // Fixed Joint
                if(currentBody.jointChildren.size() == 0)
                {
                    std::string jointName = nameOfBody + "_fixed";
                    urdfVisitor << jointName <<" being parsed."<< '\n';

                    urdfVisitor.addFixedJointAndBody(parentFrameId, bodyPose, jointName, inert, nameOfBody);
                    return;
                }

                bool composite = false;
                SE3 jointPlacement, firstJointPlacement, prevJointPlacement = SE3::Identity();

                RangeJoint rangeCompo;
                JointModelComposite jointM;
                std::string jointName;

                if(currentBody.jointChildren.size() > 1)
                {
                    composite = true;

                    MjcfJoint firstOne =  currentBody.jointChildren.at(0);
                    jointName = "Composite_" + firstOne.jointName;
                }
                
                bool isFirst = true;
                SE3 bodyInJoint;
                
                if(!composite)
                {
                    addSoloJoint(currentBody.jointChildren.at(0), currentBody);
                }
                else
                {
                    for(const auto &joint : currentBody.jointChildren)
                    {   
                        if(joint.jointType == "free")
                                throw std::invalid_argument("Joint Composite trying to be created with a freeFlyer");

                        SE3 jointInParent = bodyPose * joint.jointPlacement;
                        bodyInJoint = joint.jointPlacement.inverse();
                        if(isFirst)
                        {
                            firstJointPlacement = jointInParent;
                            jointPlacement = SE3::Identity();
                            isFirst = false;
                        }
                        else
                            jointPlacement = prevJointPlacement.inverse()*jointInParent;
                        
                        if(joint.jointType == "slide")
                        {
                            jointM.addJoint(createJoint<JointModelPX,
                                                        JointModelPY,
                                                        JointModelPZ,
                                                        JointModelPrismaticUnaligned> (joint.axis), jointPlacement);
                            
                            rangeCompo = rangeCompo.concatenate<1, 1>(joint.range);
                        }
                        else if(joint.jointType == "ball")
                        {
                            jointM.addJoint(JointModelSpherical(), jointPlacement);
                            rangeCompo = rangeCompo.concatenate<4, 3>(joint.range.setDimension<4, 3>());
                        }
                        else if(joint.jointType == "hinge")
                        {
                            jointM.addJoint(createJoint<JointModelRX,
                                                        JointModelRY,
                                                        JointModelRZ,
                                                        JointModelRevoluteUnaligned> (joint.axis), jointPlacement);
                            rangeCompo = rangeCompo.concatenate<1, 1>(joint.range);
                        }
                        else
                            throw std::invalid_argument("Unknown joint type trying to be parsed.");
                        
                        prevJointPlacement = jointInParent;
                    }
                    JointIndex joint_id;
                    const Frame & frame = urdfVisitor.model.frames[parentFrameId];

                    joint_id = urdfVisitor.model.addJoint(frame.parentJoint,
                                        jointM,
                                        firstJointPlacement,
                                        jointName,
                                        rangeCompo.maxEffort,rangeCompo.maxVel,rangeCompo.minConfig,rangeCompo.maxConfig,
                                        rangeCompo.friction,rangeCompo.damping
                                        );
                    FrameIndex jointFrameId = urdfVisitor.model.addJointFrame(joint_id, (int)parentFrameId);
                    urdfVisitor.appendBodyToJoint(jointFrameId, inert, bodyInJoint, nameOfBody);
                }
            }

            void MjcfGraph::parseRootTree()
            {

                urdfVisitor.setName(modelName);

                // get name and inertia of first root link 
                std::string rootLinkName = bodiesList.at(0);
                MjcfBody rootBody = mapOfBodies.find(rootLinkName)->second;  
                urdfVisitor.addRootJoint(rootBody.bodyInertia, rootLinkName);

                for(const auto& entry : bodiesList)
                {
                    if(entry != rootLinkName)
                        fillModel(entry);
                }
            }
        } //details
    } // mjcf
}