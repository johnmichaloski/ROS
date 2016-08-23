

#include <rviz_visual_tools/rviz_visual_tools.h>
using namespace rviz_visual_tools;

rviz_visual_tools::RvizVisualToolsPtr visual_tools;

void InitSceneObject() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
}

// INitialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix
void SetupSceneObject()
{
	Eigen::Affine3d pose= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.5, 0, 0.0);
 	Eigen::Affine3d pose4= Eigen::Affine3d::Identity()*Eigen::Translation3d(0.25, -.45, 0.0);
       //std::cout << pose << std::endl;
        Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
	Eigen::Affine3d pose2= Eigen::Affine3d::Identity()* Eigen::Translation3d(0.0, 1.0, 0.0);
	Eigen::Affine3d pose3= Eigen::Affine3d::Identity()* Eigen::Translation3d(0.0, -1.0, 0.0);
#if 0
	Eigen::Matrix3d m;
	m = AngleAxisd(0, Vector3f::UnitZ())
	    * AngleAxisd(0, Vector3f::UnitY())
	    * AngleAxisd(0, Vector3f::UnitZ());

	pose.linear() = m;
#endif
    double max_plane_size = 0.075;
    double min_plane_size = 0.01;

    bool b = visual_tools->publishXYPlane(pose1, rviz_visual_tools::TRANSLUCENT_DARK,  1.0);
    
//    b = visual_tools->publishXZPlane(pose2, rviz_visual_tools::TRANSLUCENT_LIGHT, 0.5);
//    b = visual_tools->publishXZPlane(pose3, rviz_visual_tools::TRANSLUCENT_LIGHT, .5);


    if (!(b=visual_tools->publishMesh(pose, // or const geometry_msgs::Pose &pose
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
            rviz_visual_tools::RED, // const colors &color = CLEAR,
            0.035, // double scale = 1, 
            "", // const std::string &ns = "mesh",
            1))) { //  const std::size_t &id = 0))
            std::cout << "SetupSceneObject() Failed\n";
        }
    visual_tools->triggerBatchPublish();
#if 1
    visual_tools->publishMesh(pose4, // or const geometry_msgs::Pose &pose
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            rviz_visual_tools::RED, // const colors &color = CLEAR,
            0.035, // double scale = 1, 
            "", // const std::string &ns = "mesh",
            2);
         visual_tools->triggerBatchPublish();
#endif
}
