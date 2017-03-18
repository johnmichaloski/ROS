

#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

//#include <moveit/collision_detection/world.h>
//#include <moveit/collision_detection/collision_world.h>
//#include <moveit_resources/config.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>

typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;

static boost::shared_ptr<urdf::ModelInterface> urdf_model_;
static boost::shared_ptr<srdf::Model> srdf_model_;
static robot_model::RobotModelPtr kmodel_;
static collision_detection::CollisionRobotPtr crobot_;
static collision_detection::CollisionWorldPtr cworld_;
static collision_detection::AllowedCollisionMatrixPtr acm_;
static std::string kinect_dae_resource_;
static std::string srdf_file;

void Setup(ros::NodeHandle &nh) {
  std::string urdf_xml;

  ROS_DEBUG_NAMED("ikfast", "Reading xml file from parameter server");
  if (!nh.getParam("robot_description", urdf_xml)) {
    ROS_FATAL_NAMED("IKinematics",
                    "Could not load the xml from parameter server: %s",
                    urdf_xml.c_str());
    // return false;
  }
  urdf_model_ = urdf::parseURDF(urdf_xml);
  srdf_model_.reset(new srdf::Model());
  srdf_model_->initFile(*urdf_model_, srdf_file);
  kmodel_.reset(new robot_model::RobotModel(urdf_model_, srdf_model_));
  acm_.reset(new collision_detection::AllowedCollisionMatrix(
      kmodel_->getLinkModelNames(), true));
  crobot_.reset(new DefaultCRobotType(kmodel_));
  cworld_.reset(new DefaultCWorldType());

  robot_state::RobotState kstate(kmodel_);
  kstate.setToDefaultValues();
  kstate.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  crobot_->checkSelfCollision(req, res, kstate, *acm_);
  // no collision res.collision == FALSE
}
