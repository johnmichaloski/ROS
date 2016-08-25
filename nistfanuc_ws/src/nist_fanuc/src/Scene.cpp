

#include <rviz_visual_tools/rviz_visual_tools.h>
using namespace rviz_visual_tools;

rviz_visual_tools::RvizVisualToolsPtr visual_tools;
visualization_msgs::Marker triangle_marker_;

#define XYWALL 1
#define XZWALL 2
#define YZWALL 3
bool publishWall(int WallType, const Eigen::Affine3d & inpose, const rviz_visual_tools::colors &color, double scale);


void InitSceneObject() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
    visual_tools->deleteAllMarkers();
//    visual_tools->enableBatchPublishing();
}

// INitialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix

void SetupSceneObject() {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.5, 0, 0.0);
    Eigen::Affine3d poseA = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.25, -.45, 0.0);
    Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 1.0, 0.0);
    Eigen::Affine3d pose3 = Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -1.0, 0.0);
    Eigen::Affine3d pose4 = Eigen::Affine3d::Identity() * Eigen::Translation3d(-1.0, 0.0, 0.0);
#if 0
    Eigen::Matrix3d m;
    m = AngleAxisd(0, Vector3f::UnitZ())
            * AngleAxisd(0, Vector3f::UnitY())
            * AngleAxisd(0, Vector3f::UnitZ());

    pose.linear() = m;
#endif
    double max_plane_size = 0.075;
    double min_plane_size = 0.01;
    bool b;
    // b = visual_tools->publishXYPlane(pose1, rviz_visual_tools::TRANSLUCENT_DARK, 1.0);
    //visual_tools->triggerBatchPublish();

     //   b = visual_tools->publishXZPlane(pose2, rviz_visual_tools::TRANSLUCENT_LIGHT, 0.5);
    //    b = visual_tools->publishXZPlane(pose3, rviz_visual_tools::TRANSLUCENT_LIGHT, .5);

    //visual_tools->triggerBatchPublish();
    if (!(b = visual_tools->publishMesh(pose, // or const geometry_msgs::Pose &pose
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
            rviz_visual_tools::RED, // const colors &color = CLEAR,
            0.035, // double scale = 1, 
            "mesh", 
            0))) { //  const std::size_t &id = 0))
        std::cout << "SetupSceneObject() Failed\n";
    }
    visual_tools->triggerBatchPublish();
#if 1
    visual_tools->publishMesh(poseA, // or const geometry_msgs::Pose &pose
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            rviz_visual_tools::RED, // const colors &color = CLEAR,
            0.035, // double scale = 1, 
            "mesh", 
            0);
    visual_tools->triggerBatchPublish();
#endif*
    triangle_marker_.header.frame_id = "base_link";
    triangle_marker_.ns = "Triangle";
    triangle_marker_.action = visualization_msgs::Marker::ADD;
    triangle_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangle_marker_.lifetime = ros::Duration(0.0);
    b = publishWall(XYWALL, pose1, rviz_visual_tools::TRANSLUCENT_DARK, 1);
 
   b = publishWall(XZWALL, pose2, rviz_visual_tools::TRANSLUCENT_DARK, 1);
   b = publishWall(XZWALL, pose3, rviz_visual_tools::TRANSLUCENT_DARK, 1);
   b = publishWall(YZWALL, pose4, rviz_visual_tools::TRANSLUCENT_DARK, 1);
}

bool publishWall(int WallType, const Eigen::Affine3d & inpose, const rviz_visual_tools::colors &color, double scale) {
    triangle_marker_.header.stamp = ros::Time::now();
    triangle_marker_.id++;

    triangle_marker_.color = visual_tools->getColor(color);
    triangle_marker_.pose = visual_tools->convertPose(inpose);
    geometry_msgs::Point p[4];
    if (WallType == XYWALL) {
        p[0].x = 1.0 * scale;
        p[0].y = 1.0 * scale;
        p[0].z = 0.0;

        p[1].x = -1.0 * scale;
        p[1].y = 1.0 * scale;
        p[1].z = 0.0;

        p[2].x = -1.0 * scale;
        p[2].y = -1.0 * scale;
        p[2].z = 0.0;

        p[3].x = 1.0 * scale;
        p[3].y = -1.0 * scale;
        p[3].z = 0.0;
    }
    if (WallType == XZWALL) {
        p[0].x = 1.0 * scale;
        p[0].y = 0;
        p[0].z = 1.0 * scale;

        p[1].x = -1.0 * scale;
        p[1].y = 0;
        p[1].z = 1.0 * scale;

        p[2].x = -1.0 * scale;
        p[2].y = 0;
        p[2].z = 0 * scale;

        p[3].x = 1.0 * scale;
        p[3].y = 0;
        p[3].z = 0 * scale;
    }
    if (WallType == YZWALL) {
        p[0].x = 0;
        p[0].y = 1.0 * scale;
        p[0].z = 1.0 * scale;

        p[1].x = 0;
        p[1].y = -1.0 * scale;
        p[1].z = 1.0 * scale;

        p[2].x = 0;
        p[2].y = -1.0 * scale;
        p[2].z = 0.0 * scale;

        p[3].x = 0;
        p[3].y = 1.0 * scale;
        p[3].z = 0.0 * scale;
    }
    triangle_marker_.scale.x = 1.0;
    triangle_marker_.scale.y = 1.0;
    triangle_marker_.scale.z = 1.0;

    triangle_marker_.points.clear();
    triangle_marker_.points.push_back(p[0]);
    triangle_marker_.points.push_back(p[1]);
    triangle_marker_.points.push_back(p[2]);

    triangle_marker_.points.push_back(p[2]);
    triangle_marker_.points.push_back(p[3]);
    triangle_marker_.points.push_back(p[0]);

    visual_tools->publishMarker(triangle_marker_);
    //visual_tools->markers_.markers.push_back(triangle_marker_);
    //visual_tools->triggerBatchPublish();
    return true;
}