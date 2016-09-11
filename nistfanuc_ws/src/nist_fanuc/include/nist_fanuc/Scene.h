

#pragma once
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "Globals.h"
#include <boost/bind.hpp>
#include <algorithm>
#include <map>

struct ObjectDB;

extern void DrawObject(ObjectDB *obj);
extern void InitSceneObject();
extern void ChangeColor(std::string objname, rviz_visual_tools::colors color);
extern void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color);
extern void SetupSceneObject();
extern void DrawCheckerboard();
extern bool publishCylinder(Eigen::Affine3d pose,
        rviz_visual_tools::colors color, 
        double radius, 
        double height, 
        size_t &id);

struct SonOfRvizVisualTools : public rviz_visual_tools::RvizVisualTools {

    SonOfRvizVisualTools(const std::string &base_frame ) :  rviz_visual_tools::RvizVisualTools(base_frame) {
    }

    size_t GetCylinderId() {
        return rviz_visual_tools::RvizVisualTools::cylinder_marker_.id;
    }
};

extern rviz_visual_tools::RvizVisualToolsPtr visual_tools;
//extern boost::shared_ptr<SonOfRvizVisualTools> visual_tools;

struct ObjectDB {
    Eigen::Affine3d pose;
    std::string name;
    //std::string type;
    std::string metatype;
    std::string filepath;
    rviz_visual_tools::colors color;
    double scale;
    std::size_t id;
    rviz_visual_tools::colors firstcolor;
    Eigen::Affine3d grippedpose; // where actually gripper point of object is
    double gripperclose; // displacement of gripper closed in meters
    Eigen::Affine3d adjacentpose;
    Eigen::Vector3d centroid;
    double height, radius;
    static std::map<std::string, std::string> _typemapping;

    bool King;
    ObjectDB(
            std::string name,
            std::string metatype,
             std::size_t id ) {

        this->name = name;
        this->metatype = metatype;
        this->id = id;
    }
    ObjectDB(
            std::string name,
            std::string metatype,
            std::size_t id ,
            Eigen::Affine3d pose ,
            std::string filepath ,
            rviz_visual_tools::colors color = rviz_visual_tools::CLEAR,
            double scale = 1.0) {
        this->name = name;
        this->pose = pose;
        this->metatype = metatype;
        this->filepath = filepath;
        this->firstcolor = this->color = color;
        this->scale = scale;
        if (id == 0)
            this->id = gid++;
        else this->id = id;
    }
    // cuboid
    ObjectDB(
            std::string name,
            std::string metatype,
            Eigen::Affine3d pose = Eigen::Affine3d::Identity(),
            Eigen::Affine3d adjacentpose = Eigen::Affine3d::Identity(),
            rviz_visual_tools::colors color = rviz_visual_tools::CLEAR
            ) {
        this->name = name;
        this->pose = pose;
        this->metatype = metatype;
        this->firstcolor = this->color = color;
        this->adjacentpose = adjacentpose;

    }

    // cylinder
    ObjectDB(std::string name,
            std::string metatype,
            Eigen::Affine3d pose,
            rviz_visual_tools::colors color,
            double height = 0.01,
            double radius = 0.005,
            const std::string &ns = "Cylinder") {
        this->name = name;
        this->pose = pose;
        this->metatype = metatype;
        this->firstcolor = this->color = color;
        this->height = height;
        this->radius = radius;
        this->id = 1000+gid++;
    }

    static void Save(ObjectDB * obj) {
        objects.push_back(obj);
    }

    static ObjectDB * Find(std::size_t id) {
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

    static ObjectDB * Find(std::string name) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
                boost::bind(&ObjectDB::name, _1) == name);
        if (it != objects.end())
            return *it;
        return NULL;
    }

    static std::size_t gid;
    static std::vector<ObjectDB*> objects;
    static ObjectDB * dummy;
    static std::string DumpDB()
    {
        std::stringstream str;
        for(size_t i=0; i< objects.size(); i++)
            str<< objects[i]->name << " " << objects[i]->color << "\n";
        return str.str();
               
    }
};

