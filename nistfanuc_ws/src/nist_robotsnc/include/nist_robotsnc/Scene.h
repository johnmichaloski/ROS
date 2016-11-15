

#pragma once

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <boost/bind.hpp>
#include <algorithm>
#include <map>

#include "Globals.h"
#include "RCS.h"
#include "Shape.h"
/**
 * The ObjectDB is a class to represent various RVIZ markers in the "scene". 
 * There can be a number of objects in a scene, Depending on the instance, different 
 * parameters from the class are used. The class definition is merely a union of all
 * possible parameters. Depending on the ObjectDB type a different set of parameters 
 * will be used.
 * 
 * */
struct ObjectDB {

    //bool King;
    ObjectDB() {}
    ObjectDB(
            std::string name,
            std::string metatype,
             std::size_t id )  {

        this->name = name;
        this->metatype = metatype;
        this->id = id;
    }
    std::size_t id;
    std::string name;
    //std::string type;
    std::string metatype; /**< mesh, cuboid, cylinder */
    std::string subtype; /**< solid, wireframe */
    std::string color;
    Eigen::Affine3d pose;
    Eigen::Affine3d gripperoffset;
    double gripperclosewidth; // displacement of gripper closed in meters
      
    // Mesh definitionss
    std::string filepath; /**< file path of the mesh file (STL) */
    double scale; /**< scale factor of the mesh */
    
    std::string firstcolor; // rviz_visual_tools::colors firstcolor;
    Eigen::Affine3d adjacentpose;
    Eigen::Vector3d centroid;
    double depth, width, height, radius;
    
    // Metatype name mapping into rviz_visual_tools id marker type (e.g. gear into mesh)
    static std::map<std::string, std::string> _typemapping;
    boost::shared_ptr<ShapeModel::Instance> instance;
    
};

/**
 * \brief The scene represents all the objects to be drawn in rviz using rviz_visual_tools. 
 * There can be a number of objects in a scene.
 * 
 * */
struct Scene {
    Scene();
    int MARKERCOLOR(std::string X);
    std::string  MARKERCOLOR(int X);
    bool DrawObject(ObjectDB *obj);
    void ClearScene();
    void NewScene();
    void InitScene();
    void BuildScene();
    void ChangeColor(std::string objname, std::string color); // rviz_visual_tools::colors color);
    void UpdateScene(std::string objname, Eigen::Affine3d pose, std::string color); // rviz_visual_tools::colors color);
    void DrawScene();
    void DeleteObject(std::string objname);
    bool publishCylinder(Eigen::Affine3d pose,
            std::string color,
            double radius,
            double height,
            size_t &id);

    ObjectDB * CreateMesh(
            std::string name,
            std::string metatype,
            std::size_t id,
            Eigen::Affine3d pose,
            std::string filepath,
            std::string color= "CLEAR", 
            double scale = 1.0) ;
    // cuboid
    ObjectDB * CreateCuboid(
            std::string name,
            std::string metatype,
            Eigen::Affine3d pose = Eigen::Affine3d::Identity(),
            Eigen::Affine3d adjacentpose = Eigen::Affine3d::Identity(),
             std::string color= "CLEAR" 
            );
    // cylinder
    ObjectDB * Scene::CreateCylinder(std::string name,
            std::string metatype,
            Eigen::Affine3d pose,
            std::string color, //rviz_visual_tools::colors color,
            double height,
            double radius,
            const std::string &ns);
    
    // x cross marker
    ObjectDB * CreateMarker(std::string name,
            Eigen::Affine3d pose,
             std::string color) ;
    
    // wireframe cuboid
    ObjectDB* CreateWireframeCuboid(std::string name,
            std::string metatype,
            Eigen::Affine3d pose,
            double depth = 0.005,
            double width = 0.005,
            double height = 0.01,
            std::string color= "GREEN" , 
            const std::string &ns = "WireframeCuboid") ;

    void Save(ObjectDB * obj);

    ObjectDB * Find(std::size_t id) ;

    Eigen::Affine3d& FindPose(std::string name);

    ObjectDB * Find(std::string name);

    std::string DumpDB() ;
    
    static ObjectDB * dummy;
    static std::size_t gid;
    std::vector<ObjectDB*> objects;
    //Eigen::Affine3d fanucoffset00;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    //extern boost::shared_ptr<SonOfRvizVisualTools> visual_tools;// did not work
    visualization_msgs::Marker triangle_marker_;
    visualization_msgs::Marker cylinder_marker_;
    //std::vector<tf::Pose> gearspots;
    //std::vector<tf::Pose> sku_gear_vessel;  // fixme needs description of gear size to match

};
extern boost::shared_ptr<Scene> pScene;

struct SonOfRvizVisualTools : public rviz_visual_tools::RvizVisualTools {

    SonOfRvizVisualTools(const std::string &base_frame ) :  rviz_visual_tools::RvizVisualTools(base_frame) {
    }

    size_t GetCylinderId() {
        return rviz_visual_tools::RvizVisualTools::cylinder_marker_.id;
    }
};



