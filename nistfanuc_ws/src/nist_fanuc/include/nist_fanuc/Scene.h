

#include <rviz_visual_tools/rviz_visual_tools.h>
#include "Globals.h"
#include <boost/bind.hpp>
#include <algorithm>

struct ObjectDB;

#define XYWALL 1
#define XZWALL 2
#define YZWALL 3

extern bool publishWall(int WallType, const Eigen::Affine3d & inpose, const rviz_visual_tools::colors &color, double scale);
extern void DrawObject(ObjectDB *obj);
extern void InitSceneObject();
extern void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color );
extern void SetupSceneObject();

struct Wall {

    Wall(int type = 0, double length = 0.0, double width = 0.0, double height = 0.0) {
        this->type = type;
        this->length = length;
        this->height = height;
        this->width = width;
    }
    int type;
    double length, height, width;
};  
struct ObjectDB
{
    Eigen::Affine3d pose;
    std::string name;
    std::string type;
    std::string filepath;
    rviz_visual_tools::colors color;
    double scale;
    std::size_t id;
     rviz_visual_tools::colors firstcolor;
    Eigen::Affine3d grippedpose; // where actually gripper point of object is
    double gripperclose; // displacement of gripper closed in meters


    // Wall specifics
 
    Wall wall;
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
        this->firstcolor=this->color = color;
        this->scale = scale;
        if(id==0)
            this->id = gid++;
        else this->id = id;
     }
ObjectDB(
            Wall wall,
            std::string name,
            std::string type,
            std::size_t id = 0,
            Eigen::Affine3d pose = Eigen::Affine3d::Identity(),
            rviz_visual_tools::colors color = rviz_visual_tools::CLEAR,
            double scale = 1.0
            ) {
        this->name = name;
        this->pose = pose;
        this->type = type;
        this->firstcolor=this->color = color;
        this->scale = scale;
        this->wall=wall;

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

