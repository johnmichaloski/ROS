
#include "Scene.h"

#include "boost/assign.hpp"
using namespace std;
using namespace boost::assign;

#include "BLogging.h"
#include "Conversions.h"
#include "Debug.h"
using namespace rviz_visual_tools;

std::map<std::string,std::string> ObjectDB::_typemapping =  map_list_of  ("bolt", "mesh") ("boltholder", "mesh") ("wall", "cuboid") ;
        
std::size_t ObjectDB::gid = 1;
std::vector<ObjectDB*> ObjectDB::objects;
ObjectDB * ObjectDB::dummy = new ObjectDB("dummy", "nevermatch", (std::size_t) 0);

rviz_visual_tools::RvizVisualToolsPtr visual_tools;
visualization_msgs::Marker triangle_marker_;

void InitSceneObject() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
    //visual_tools->deleteAllMarkers();
    //    visual_tools->enableBatchPublishing();
    //visual_tools->waitForMarkerPub();
    //visual_tools->loadMarkerPub(true,false);
    Globals.Sleep(10000);

    // Setup up triangle_marker_ for wall drawing
    triangle_marker_.header.frame_id = "base_link";
    triangle_marker_.ns = "Triangle";
    triangle_marker_.action = visualization_msgs::Marker::ADD;
    triangle_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangle_marker_.lifetime = ros::Duration(0.0);


    ObjectDB * obj;

#if 1
    ObjectDB::Save(obj = new ObjectDB(
            "rightwall", "wall", 
             Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0.501, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
    ObjectDB::gid++;
#endif    
           ObjectDB::Save(obj = new ObjectDB(
            "backwall", "wall", 
             Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.0,  1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.501,  -1.0,  0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );  
           ObjectDB::gid++;
#if 0
        ObjectDB::Save(obj = new ObjectDB(
            "leftwall", "wall", 
             Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -0.5, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, -0.501, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
           ObjectDB::Save(obj = new ObjectDB(
            "backwall", "wall", 
             Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, -1.0,  1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5,  1.01,  1.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
           
#endif

    // Scene bolt and boltholder objects
    ObjectDB::Save(new ObjectDB("boltholder1", "boltholder", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.5, 0, 0.0),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
            rviz_visual_tools::RED, 0.035));
    ObjectDB::Save(new ObjectDB("bolt1", "bolt", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.25, -.45, 0.04),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            rviz_visual_tools::RED, 0.035));

}

// Initialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix

void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color) {
    ObjectDB * obj = ObjectDB::Find(objname);
    obj->color = color;
    obj->pose = pose;
    DrawObject(obj);

}

void DrawObject(ObjectDB *obj) {
    bool b;
    std::string type = ObjectDB::_typemapping[obj->metatype];
    if (type == "mesh") {
        b=visual_tools->publishMesh(obj->pose,
                obj->filepath, 
                obj->color, 
                obj->scale, 
                type,  
                obj->id);
    } else if (type == "cuboid") {
//        visual_tools->publishCuboid(obj->pose.translation(), 
//                obj->adjacentpose.translation(), 
//                obj->color); // Eigen::Vector3d(0.0, 0.5, 1.0), 
    }
    BOOST_ASSERT_MSG(b>0, "Failed to publish object");
    ros::spinOnce();
    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();
    ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();

}

void SetupSceneObject() {
    for (size_t i = 0; i < ObjectDB::objects.size(); i++) {
        ObjectDB *obj = ObjectDB::objects[i];
        DrawObject(obj);
    }

}

void DrawCheckerboard() {
    return;
    double xoffset = 1.0;
    double rowoffset=0.04;
    double yoffset = -0.5;
    double offset = 0.04;
    for (size_t row = 0; row < 8; row++) {
        
        double rowoffset = xoffset+ (offset*row);         
        for (size_t i = 0; i <=8; i = i + 2) {
            double coloffset=yoffset + (i*offset);
            if(row%2==0) coloffset=coloffset+offset; // red offset at zero
            
            Eigen::Vector3d up(rowoffset, coloffset, 0.01);
            Eigen::Vector3d down(rowoffset + offset, coloffset + offset, 0.0);

            visual_tools->publishCuboid(up, down, rviz_visual_tools::WHITE);
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::Duration(0.5).sleep();

            if (row % 2 == 0) coloffset = coloffset - offset; // red offset at zero
            else coloffset = coloffset + offset;
            Eigen::Vector3d bup(rowoffset, coloffset, 0.01);
            Eigen::Vector3d bdown(rowoffset + offset, coloffset + offset, 0.0);
            visual_tools->publishCuboid(bup, bdown,
                    rviz_visual_tools::BLACK);
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
        }
    }

}