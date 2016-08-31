
#include "Scene.h"
using namespace rviz_visual_tools;

#if 0
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "Globals.h"
#include <boost/bind.hpp>
#include <algorithm>
struct ObjectDB;

#define XYWALL 1
#define XZWALL 2
#define YZWALL 3

bool publishWall(int WallType, const Eigen::Affine3d & inpose, const rviz_visual_tools::colors &color, double scale);
void DrawObject(ObjectDB *obj);
void InitSceneObject();
void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color );
void SetupSceneObject() ;

struct ObjectDB
{
    Eigen::Affine3d pose;
    std::string name;
    std::string type;
    std::string filepath;
    rviz_visual_tools::colors color;
    double scale;
    std::size_t id;
    int walltype;
     ObjectDB(
            std::string name,
            std::string type,
            std::size_t id=0,
            Eigen::Affine3d pose=Eigen::Affine3d::Identity(),
            std::string filepath="",
            rviz_visual_tools::colors color=rviz_visual_tools::CLEAR,
            double scale = 1.0) {
        this->name = name;
        this->pose = pose;
        this->type = type;
        this->filepath = filepath;
        this->color = color;
        this->scale = scale;
        if(id==0)
            this->id = gid++;
        else this->id = id;
     }

     static void Save( ObjectDB * obj) { objects.push_back(obj); }

     static ObjectDB * Find( std::size_t id) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
                boost::bind(&ObjectDB::id, _1) == id);

        if (it != objects.end())
            return *it;

        return NULL;
     }

     static Eigen::Affine3d& FindPose(std::string name) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
                boost::bind(&ObjectDB::name, _1) == name);
        if (it != objects.end())
            return (*it)->pose;
        return dummy->pose;
    }
     static ObjectDB * Find( std::string name) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(), 
        boost::bind(&ObjectDB::name, _1) == name); 
        if(it!=objects.end())
            return *it;
        return NULL;
     }

   static std::size_t gid;
   static  std::vector<ObjectDB*> objects;
   static ObjectDB * dummy;
};

#endif

std::size_t ObjectDB::gid=1;
std::vector<ObjectDB*> ObjectDB::objects;
ObjectDB * ObjectDB::dummy= new ObjectDB("dummy","nevermatch");

rviz_visual_tools::RvizVisualToolsPtr visual_tools;
visualization_msgs::Marker triangle_marker_;



void InitSceneObject() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("base_link", "/visualization_marker_array"));
    visual_tools->deleteAllMarkers();
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
   
    // Scene objects
    ObjectDB::Save(new ObjectDB("boltholder", "mesh", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.5, 0, 0.0),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear_holder.stl",
            rviz_visual_tools::RED, 0.035));
    ObjectDB::Save(new ObjectDB("bolt", "mesh", ObjectDB::gid++,  Eigen::Affine3d::Identity() * Eigen::Translation3d(0.25, -.45, 0.0),
            "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            rviz_visual_tools::RED, 0.035));
    
    ObjectDB * obj;
    ObjectDB::Save(obj = new ObjectDB("floor", "wall", ObjectDB::gid++, Eigen::Affine3d::Identity(),
            "", rviz_visual_tools::TRANSLUCENT_DARK, 1.0));
    obj->walltype = XYWALL;
    ObjectDB::Save(obj = new ObjectDB("rightwall", "wall", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 1.0, 0.0),
            "", rviz_visual_tools::TRANSLUCENT_DARK, 1.0));
    obj->walltype = XZWALL;
        ObjectDB::Save(obj = new ObjectDB("leftwall", "wall", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -1.0, 0.0),
            "", rviz_visual_tools::TRANSLUCENT_DARK, 1.0));
    obj->walltype = XZWALL;
        ObjectDB::Save(obj = new ObjectDB("backwall", "wall", ObjectDB::gid++, Eigen::Affine3d::Identity() * Eigen::Translation3d(-1.0, 0.0, 0.0),
            "", rviz_visual_tools::TRANSLUCENT_DARK, 1.0));
    obj->walltype = YZWALL;
    
}

// INitialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix

void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color )
{
    ObjectDB * obj  = ObjectDB::Find( objname);
    obj->color=color;
    obj->pose=pose;
    DrawObject(obj);
   
}

void DrawObject(ObjectDB *obj) {
    bool b;
    if (obj->type == "mesh")
        visual_tools->publishMesh(obj->pose,
            obj->filepath, // "file:///usr/local/michalos/nistfanuc_ws/src/nist_fanuc/worldmodel/medium_gear.stl",
            obj->color, // rviz_visual_tools::RED, // const colors &color = CLEAR,
            obj->scale, //  0.035, // double scale = 1, 
            obj->type, // "mesh", 
            obj->id);
    else if (obj->type == "wall")
        b = publishWall(obj->walltype, obj->pose, obj->color, 1);

}
void SetupSceneObject() {
    for(size_t i=0; i< ObjectDB::objects.size(); i++){
        ObjectDB *obj = ObjectDB::objects[i];
        DrawObject(obj);
    }
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