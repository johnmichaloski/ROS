// David Lu of WU Source Code bsd license blah blah


#include "arm_kinematics.h"
using std::string;

static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";


//Kinematics::Kinematics(): nh_private ("~") {
Kinematics::Kinematics() {
}

bool Kinematics::init(ros::NodeHandle &nh, std::string tipname, std::string rootname) {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    nh.param("urdf_xml",urdf_xml,std::string("robot_description"));
    nh.searchParam(urdf_xml,full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh.getParam(full_urdf_xml, result)) {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }
#if 0
    // Get Root and Tip From Parameter Service
    if (!nh.getParam("root_name", root_name)) {
        ROS_FATAL("GenericIK: No root name found on parameter server");
        return false;
    }
    if (!nh.getParam("tip_name", tip_name)) {
        ROS_FATAL("GenericIK: No tip name found on parameter server");
        return false;
    }
#endif
    //nh.getParam("root_name", root_name);
    //nh.getParam("tip_name", tip_name);
    // These must be passed in as this class is instantiated numerous times, and using nh more difficult
     root_name=rootname;
     tip_name=tipname;
     
    // Load and Read Models
    if (!loadModel(result)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

   // Get Solver Parameters
    int maxIterations;
    double epsilon;
    
    nh.param("maxIterations", maxIterations, 10000);
    nh.param("epsilon", epsilon, 1e-2);


 

    // Build Solvers
    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
    ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max,
            *fk_solver, *ik_solver_vel, maxIterations, epsilon);

#if 0
    ROS_INFO("Advertising services");
    fk_service = nh.advertiseService(FK_SERVICE,&Kinematics::getPositionFK,this);
    ik_service = nh.advertiseService(IK_SERVICE,&Kinematics::getPositionIK,this);
    ik_solver_info_service = nh.advertiseService(IK_INFO_SERVICE,&Kinematics::getIKSolverInfo,this);
    fk_solver_info_service = nh.advertiseService(FK_INFO_SERVICE,&Kinematics::getFKSolverInfo,this);
#endif
    return true;
}

bool Kinematics::loadModel(const std::string xml) {
    // http://wiki.ros.org/urdf/Tutorials/Parse%20a%20urdf%20file
    KDL::Tree tree;

    if (!robot_model.initString(xml)) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    if (!tree.getChain(root_name, tip_name, chain)) {
        ROS_ERROR("Could not initialize chain object");
        return false;
    }

    if (!readJoints(robot_model)) {
        ROS_FATAL("Could not read information about the joints");
        return false;
    }

    return true;
}

bool Kinematics::readJoints(urdf::Model &robot_model) {
    num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;

    while (link && link->name != root_name) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (!joint) {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    info.joint_names.resize(num_joints);
    info.link_names.resize(num_joints);
    info.limits.resize(num_joints);

    link = robot_model.getLink(tip_name);
    unsigned int i = 0;
    while (link && i < num_joints) {
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
            ROS_DEBUG( "getting bounds for joint: [%s]", joint->name.c_str() );

            float lower, upper;
            int hasLimits;
            if ( joint->type != urdf::Joint::CONTINUOUS ) {
                lower = joint->limits->lower;
                upper = joint->limits->upper;
                hasLimits = 1;
            } else {
                lower = -M_PI;
                upper = M_PI;
                hasLimits = 0;
            }
            int index = num_joints - i -1;

            joint_min.data[index] = lower;
            joint_max.data[index] = upper;
            info.joint_names[index] = joint->name;
            info.link_names[index] = link->name;
            info.limits[index].joint_name = joint->name;
            info.limits[index].has_position_limits = hasLimits;
            info.limits[index].min_position = lower;
            info.limits[index].max_position = upper;
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}


int Kinematics::getJointIndex(const std::string &name) {
    for (unsigned int i=0; i < info.joint_names.size(); i++) {
        if (info.joint_names[i] == name)
            return i;
    }
    return -1;
}

int Kinematics::getKDLSegmentIndex(const std::string &name) {
    int i=0; 
    while (i < (int)chain.getNrOfSegments()) {
        if (chain.getSegment(i).getName() == name) {
            return i+1;
        }
        i++;
    }
    return -1;
}

bool Kinematics::getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                               moveit_msgs::GetPositionIK::Response &response) {

    geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
    tf::Stamped<tf::Pose> transform;
    tf::Stamped<tf::Pose> transform_root;
    tf::poseStampedMsgToTF( pose_msg_in, transform );

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(num_joints);
    jnt_pos_out.resize(num_joints);
    for (unsigned int i=0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.ik_request.robot_state.joint_state.name[i]   ) ;//.ik_request.ik_seed_state.joint_state.name[i]);
        if (tmp_index >=0) {
            jnt_pos_in(tmp_index) = request.ik_request.robot_state.joint_state.position[i];
        } else {
            ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.robot_state.joint_state.name[i].c_str());
        }
    }
    KDL::Frame F_dest;

#if 0
    //Convert F to our root_frame
    try {
        tf_listener.transformPose(root_name, transform, transform_root);
    } catch (...) {
        ROS_ERROR("Could not transform IK pose to frame: %s", root_name.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
       return false;
    }
   tf::TransformTFToKDL(transform_root, F_dest);
#else
    tf::TransformTFToKDL(transform, F_dest);
#endif
    try {

        int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

        if (ik_valid >= 0) {
            response.solution.joint_state.name = info.joint_names;
            response.solution.joint_state.position.resize(num_joints);
            for (unsigned int i = 0; i < num_joints; i++) {
                response.solution.joint_state.position[i] = jnt_pos_out(i);
                ROS_DEBUG("IK Solution: %s %d: %f", response.solution.joint_state.name[i].c_str(), i, jnt_pos_out(i));
            }
            response.error_code.val = response.error_code.SUCCESS;
            return true;
        } else {
            ROS_DEBUG("An IK solution could not be found");
            response.error_code.val = response.error_code.NO_IK_SOLUTION;
            return true;
        }
    }    catch (...) {
        
    }
    ROS_DEBUG("An IK solution exception was thrown");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return true;
}

bool Kinematics::getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool Kinematics::getFKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request &request,
                                 moveit_msgs::GetKinematicSolverInfo::Response &response) {
    response.kinematic_solver_info = info;
    return true;
}

bool Kinematics::getPositionFK(moveit_msgs::GetPositionFK::Request &request,
                               moveit_msgs::GetPositionFK::Response &response) {
    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;
     bool valid = true;
   
    // I see you give a FK for each joint not just at ee !!

    response.pose_stamped.resize(1); //request.fk_link_names.size());
    response.fk_link_names.resize(1); // request.fk_link_names.size());

    // Create joint array
    unsigned int nj = request.fk_link_names.size();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    for (unsigned int i=0; i < nj; i++) {
        jointpositions(i)=(double)request.robot_state.joint_state.position[i];
    }

    if (fk_solver->JntToCart(jointpositions, p_out) >= 0) {
        tf_pose.frame_id_ = root_name;
        tf_pose.stamp_ = ros::Time();
        tf::PoseKDLToTF(p_out, tf_pose);
        tf::poseStampedTFToMsg(tf_pose, pose);
        response.pose_stamped[0] = pose;
        response.fk_link_names[0] = request.fk_link_names[5];
        response.error_code.val = response.error_code.SUCCESS;
    } else {
        ROS_ERROR("Could not compute FK for %s", request.fk_link_names[0].c_str());
        response.error_code.val = response.error_code.FAILURE;
        valid = false;
    }

    return true;
}
#if 0
    jnt_pos_in.resize(num_joints);
    for (unsigned int i = 0; i < num_joints; i++) {
        int tmp_index = getJointIndex(request.robot_state.joint_state.name[i]);
        if (tmp_index >= 0)
            jnt_pos_in(tmp_index) = request.robot_state.joint_state.position[i];
    }


    for (unsigned int i = 0; i < request.fk_link_names.size(); i++) {
        int segmentIndex = getKDLSegmentIndex(request.fk_link_names[i]);
        ROS_DEBUG("End effector index: %d", segmentIndex);
        ROS_DEBUG("Chain indices: %d", chain.getNrOfSegments());
        if (fk_solver->JntToCart(jnt_pos_in, p_out, segmentIndex) >= 0) {
            tf_pose.frame_id_ = root_name;
            tf_pose.stamp_ = ros::Time();
            tf::PoseKDLToTF(p_out, tf_pose);
            // http://docs.ros.org/diamondback/api/tf/html/c++/classtf_1_1TransformListener.html#ae5e3a12c3cd66f250f1d717c017ce524
            // not sure what frame_id does?
            //            try {
            //                tf_listener.transformPose(request.header.frame_id,tf_pose,tf_pose);
            //            } catch (...) {
            //                ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
            //                response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
            //                return false;
            //            }
            tf::poseStampedTFToMsg(tf_pose, pose);
            response.pose_stamped[i] = pose;
            response.fk_link_names[i] = request.fk_link_names[i];
            response.error_code.val = response.error_code.SUCCESS;
        } else {
            ROS_ERROR("Could not compute FK for %s", request.fk_link_names[i].c_str());
            response.error_code.val = response.error_code.FAILURE;
            valid = false;
        }
    }
    return true;
}
#endif

#if 0
int main(int argc, char **argv) {
   SetupRosEnvironment("");
    ros::init(argc, argv, "arm_kinematics");
     Kinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }

    ros::spin();
    return 0;
}
#endif
