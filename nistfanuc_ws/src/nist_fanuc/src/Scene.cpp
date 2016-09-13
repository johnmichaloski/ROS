
#include "Scene.h"

#include "boost/assign.hpp"
using namespace std;
using namespace boost::assign;

#include "BLogging.h"
#include "Conversions.h"
#include "Debug.h"
using namespace rviz_visual_tools;


std::map<std::string,std::string> ObjectDB::_typemapping = 
        map_list_of  ("bolt", "mesh") 
        ("boltholder", "mesh")
        ("wall", "cuboid") 
        ("Checkerboard", "cuboid") 
        ("Cylinder", "Cylinder") 
         ;
        
std::size_t ObjectDB::gid = 1;
std::vector<ObjectDB*> ObjectDB::objects;
ObjectDB * ObjectDB::dummy = new ObjectDB("dummy", "nevermatch", (std::size_t) 0);

rviz_visual_tools::RvizVisualToolsPtr visual_tools;
//boost::shared_ptr<SonOfRvizVisualTools> visual_tools;
visualization_msgs::Marker triangle_marker_;
visualization_msgs::Marker cylinder_marker_;

void InitSceneObject() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
    //visual_tools = boost::shared_ptr<SonOfRvizVisualTools>(new SonOfRvizVisualTools("base_link"));
    
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
    // Load Cylinder ----------------------------------------------------
    cylinder_marker_.header.frame_id = "base_link";
     cylinder_marker_.ns = "Cylinder";
    cylinder_marker_.action = visualization_msgs::Marker::ADD;
    cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker_.lifetime = ros::Duration(0.0);
    cylinder_marker_.id=1;
    ObjectDB * obj;

#if 0
    ObjectDB::Save(obj = new ObjectDB(
            "rightwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0.501, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
    ObjectDB::gid++;

    ObjectDB::Save(obj = new ObjectDB(
            "backwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.0, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.501, -1.0, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
    ObjectDB::gid++;
#endif 
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

#ifdef BOLTDEMO
    // Scene bolt and boltholder objects
    ObjectDB::Save(new ObjectDB("boltholder1", "boltholder", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.5, 0, 0.0),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
            rviz_visual_tools::RED, 0.035));
    ObjectDB::Save(new ObjectDB("bolt1", "bolt", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.25, -.45, 0.04),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            rviz_visual_tools::RED, 0.035));
#endif

}

// Initialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix

void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color) {
    ObjectDB * obj = ObjectDB::Find(objname);
    if(obj==NULL) 
        throw std::runtime_error("Gak UpdateScene!");
    obj->color = color;
    obj->pose = pose;
    DrawObject(obj);
    visual_tools->triggerBatchPublish ();

}

void DeleteObject(std::string objname) {
    ObjectDB * obj = ObjectDB::Find(objname);
    if (obj == NULL)
        throw std::runtime_error("Gak DeleteObject!");
    cylinder_marker_.action = visualization_msgs::Marker::DELETE;
    DrawObject(obj);
    visual_tools->triggerBatchPublish();
    cylinder_marker_.action = visualization_msgs::Marker::ADD;
}

void ChangeColor(std::string objname, rviz_visual_tools::colors color) {
    ObjectDB * obj = ObjectDB::Find(objname);
    if (obj == NULL)
        throw std::runtime_error("Gak ChangeColor!");
    obj->color = color;
    cylinder_marker_.action = visualization_msgs::Marker::MODIFY;
    DrawObject(obj);
    visual_tools->triggerBatchPublish ();

    cylinder_marker_.action = visualization_msgs::Marker::ADD;

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
    } 
#if 1
    else if (type == "cuboid") {
        b=visual_tools->publishCuboid(obj->pose.translation(), 
                obj->adjacentpose.translation(), 
                obj->color); // Eigen::Vector3d(0.0, 0.5, 1.0), 
    }
#endif
#if 1
    else if ( type == "Cylinder") {
//        visual_tools->publishCylinder(obj->pose, 
//                obj->color, 
//                obj-> height, 
//                obj-> radius,
//                "Cylinder");
       publishCylinder(obj->pose,
                obj->color,
                obj-> height,
                obj-> radius,
                obj->id); 

    }
#endif          
    //BOOST_ASSERT_MSG(b>0, "Failed to publish object");
    ros::spinOnce();
    //ros::Duration(0.5).sleep(); // sleep for half a second
    ros::spinOnce();
//    ros::Duration(0.5).sleep(); // sleep for half a second
//    ros::spinOnce();
    visual_tools->triggerBatchPublish ();

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

bool publishCylinder(Eigen::Affine3d pose,
        rviz_visual_tools::colors color, 
        double height, 
        double radius, 
        size_t &id)
{
#if 0
// Distance between two points
  double height = (point1 - point2).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = getCenterPoint(point1, point2);

  // Create vector
  Eigen::Affine3d pose;
  pose = getVectorBetweenPoints(pt_center, point2);
  // Convert pose to be normal to cylindar axis
  Eigen::Affine3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;
  #endif

   // Set the timestamp
  cylinder_marker_.header.stamp = ros::Time::now();
  cylinder_marker_.ns = "Cylinder";
  //id=cylinder_marker_.id;
  cylinder_marker_.id=id; // ++;

  // Set the pose
  cylinder_marker_.pose = visual_tools->convertPose(pose);

  // Set marker size
  cylinder_marker_.scale.x = radius;
  cylinder_marker_.scale.y = radius;
  cylinder_marker_.scale.z = height;

  // Set marker color
  cylinder_marker_.color = visual_tools->getColor(color);

  // Helper for publishing rviz markers
  return visual_tools->publishMarker(cylinder_marker_);
}