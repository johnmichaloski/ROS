
#include "CollisionDetection.h"
#include <fstream>
#include <moveit/planning_scene/planning_scene.h>

void CDetectCollision::Setup(ros::NodeHandle &nh, std::string urdf_file, std::string srdf_file) {
    std::string xml_string;
    std::fstream xml_file(urdf_file.c_str(), std::fstream::in);

    if (xml_file.is_open()) {
        while (xml_file.good()) {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        urdf_model_ = urdf::parseURDF(xml_string);
        urdf_ok_ = urdf_model_!=NULL;
    } else {
        ROS_ERROR("FAILED TO OPEN URDF FILE: %s", urdf_file.c_str());
        urdf_ok_ = false;
        return;
    }

    
  srdf_model_.reset(new srdf::Model());
  srdf_model_->initFile(*urdf_model_, srdf_file);
  kmodel_.reset(new robot_model::RobotModel(urdf_model_, srdf_model_));
  acm_.reset(new collision_detection::AllowedCollisionMatrix(
      kmodel_->getLinkModelNames(), true));
  crobot_.reset(new DefaultCRobotType(kmodel_));
  cworld_.reset(new DefaultCWorldType());

}
#include "Debug.h"

bool CDetectCollision::SanityCheck() {
 
   robot_state::RobotState kstate(kmodel_);
//    planning_scene::PlanningScene planning_scene(kmodel_);
//    const robot_state::RobotState &kstate = planning_scene.getCurrentStateNonConst();
    kstate.setToDefaultValues();
    kstate.update();

    ROS_INFO(RobStateDump(kstate).c_str());
    ROS_INFO(ACMDump(acm_).c_str());
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.verbose = true;
    req.distance = true;
    res.clear();
    
//    planning_scene.checkSelfCollision(req, res);
    crobot_->checkSelfCollision(req, res,  kstate, *acm_);
    // no collision res.collision == FALSE
    return res.collision == false;
}

// http://docs.ros.org/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
bool CDetectCollision::RobotCollisionCheck(std::map< std::string, double > joint_state_map) {
    planning_scene::PlanningScene planning_scene(kmodel_);
    const robot_state::RobotState &kstate = planning_scene.getCurrentStateNonConst();
    sensor_msgs::JointState msg;

    kstate.setVariablePositions(joint_state_map);
    kstate.update();
    ROS_INFO(RobStateDump(kstate).c_str());

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.verbose=true;
    req.distance = true;
    res.clear();
    planning_scene.checkSelfCollision(req, res);
    // no collision res.collision == false
    return res.collision == false;
    
}