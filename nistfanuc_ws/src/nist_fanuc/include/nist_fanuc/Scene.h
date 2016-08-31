

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
extern void SetupSceneObject() ;

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

