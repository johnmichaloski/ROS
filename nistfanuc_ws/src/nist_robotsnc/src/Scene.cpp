
/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 
#include "Scene.h"

#include "boost/assign.hpp"
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::assign;

#include "Boost.h"
#include "Conversions.h"
#include "Debug.h"
#include "Config.h"
#include "Shape.h"
using namespace rviz_visual_tools;


std::map<std::string, std::string> ObjectDB::_typemapping =
        map_list_of("gear", "mesh")
("gearholder", "mesh")
("wall", "cuboid")
("Checkerboard", "cuboid")
("Cylinder", "Cylinder")
("trayoutline", "WireframeCuboid")
("marker", "Mark")
;

std::size_t Scene::gid = 1;
//std::vector<ObjectDB*> Scen::objects;
ObjectDB * Scene::dummy = new ObjectDB("dummy", "nevermatch", (std::size_t) 0);

int Scene::MARKERCOLOR(std::string X) {
    boost::to_upper(X);
    if (X == "RED") return rviz_visual_tools::RED;
    if (X == "GREEN") return rviz_visual_tools::GREEN;
    if (X == "CLEAR") return rviz_visual_tools::CLEAR;
    if (X == "BLACK") return rviz_visual_tools::BLACK;
    if (X == "BROWN") return rviz_visual_tools::BROWN;
    if (X == "BLUE") return rviz_visual_tools::BLUE;
    if (X == "CYAN") return rviz_visual_tools::CYAN;
    if (X == "GREY") return rviz_visual_tools::GREY;
    if (X == "DARK_GREY") return rviz_visual_tools::DARK_GREY;
    if (X == "GREEN") return rviz_visual_tools::GREEN;
    if (X == "LIME_GREEN") return rviz_visual_tools::LIME_GREEN;
    if (X == "MAGENTA") return rviz_visual_tools::MAGENTA;
    if (X == "ORANGE") return rviz_visual_tools::ORANGE;
    if (X == "PURPLE") return rviz_visual_tools::PURPLE;
    if (X == "PINK") return rviz_visual_tools::PINK;
    if (X == "WHITE") return rviz_visual_tools::WHITE;
    if (X == "YELLOW") return rviz_visual_tools::YELLOW;
    if (X == "YELLOW") return rviz_visual_tools::YELLOW;
    if (X == "TRANSLUCENT") return rviz_visual_tools::TRANSLUCENT;
    if (X == "TRANSLUCENT_LIGHT") return rviz_visual_tools::TRANSLUCENT_LIGHT;
    if (X == "TRANSLUCENT_DARK") return rviz_visual_tools::TRANSLUCENT_DARK;
    if (X == "CLEAR") return rviz_visual_tools::CLEAR;


    return rviz_visual_tools::DEFAULT;
}
std::string  Scene::MARKERCOLOR(int X) {
    if (X ==  rviz_visual_tools::RED) return "RED";
    if (X ==  rviz_visual_tools::GREEN) return "GREEN";
    if (X == rviz_visual_tools::CLEAR) return "CLEAR";
    if (X == rviz_visual_tools::BLACK) return "BLACK" ;
    if (X == rviz_visual_tools::BROWN) return "BROWN" ;
    if (X == rviz_visual_tools::BLUE) return "BLUE" ;
    if (X == rviz_visual_tools::CYAN) return "CYAN" ;
    if (X == rviz_visual_tools::GREY) return "GREY" ;
    if (X == rviz_visual_tools::DARK_GREY) return "DARK_GREY" ;
    if (X == rviz_visual_tools::GREEN) return "GREEN" ;
    if (X == rviz_visual_tools::LIME_GREEN) return "LIME_GREEN" ;
    if (X == rviz_visual_tools::MAGENTA) return "MAGENTA" ;
    if (X == rviz_visual_tools::ORANGE) return "ORANGE" ;
    if (X == rviz_visual_tools::PURPLE) return "PURPLE" ;
    if (X == rviz_visual_tools::PINK) return "PINK" ;
    if (X == rviz_visual_tools::WHITE) return "WHITE" ;
    if (X == rviz_visual_tools::YELLOW) return "YELLOW" ;
    if (X == rviz_visual_tools::TRANSLUCENT) return "TRANSLUCENT" ;
    if (X == rviz_visual_tools::TRANSLUCENT_LIGHT) return "TRANSLUCENT_LIGHT" ;
    if (X == rviz_visual_tools::TRANSLUCENT_DARK) return "TRANSLUCENT_DARK" ;
    if (X == rviz_visual_tools::CLEAR) return "CLEAR"  ;
    return "DEFAULT";
}
boost::shared_ptr<Scene> pScene = boost::shared_ptr<Scene>(new Scene());

ObjectDB * Scene::CreateMesh(
        std::string name,
        std::string metatype,
        std::size_t id,
        Eigen::Affine3d pose,
        std::string filepath,
        std::string color, // rviz_visual_tools::colors color,
        double scale) {
    ObjectDB* obj = new ObjectDB();
    obj->name = name;
    obj->pose = pose;
    obj->metatype = metatype;
    obj->filepath = filepath;
    obj->firstcolor = obj->color = color;
    obj->scale = scale;
    if (id == 0)
        obj->id = gid++;
    else obj->id = id;
    Save(obj);
    return obj;
}
// cuboid

ObjectDB * Scene::CreateCuboid(
        std::string name,
        std::string metatype,
        Eigen::Affine3d pose,
        Eigen::Affine3d adjacentpose,
        std::string color // rviz_visual_tools::colors color
        ) {
    ObjectDB* obj = new ObjectDB();
    obj->name = name;
    obj->pose = pose;
    obj->metatype = metatype;
    obj->firstcolor = obj->color = color;
    obj->adjacentpose = adjacentpose;
    Save(obj);
    return obj;
}
// marker
ObjectDB * Scene::CreateMarker(std::string name,
        Eigen::Affine3d pose,
        std::string color) {
    ObjectDB* obj = new ObjectDB();
    obj->name = name;
    obj->pose = pose;
    obj->metatype = "marker";
    obj->firstcolor = obj->color = color;
    obj->id = gid++;
    Save(obj);
    return obj;
}
// cylinder

ObjectDB * Scene::CreateCylinder(std::string name,
        std::string metatype,
        Eigen::Affine3d pose,
        std::string color, //rviz_visual_tools::colors color,
        double height,
        double radius,
        const std::string &ns) {
    ObjectDB* obj = new ObjectDB();
    obj->name = name;
    obj->pose = pose;
    obj->metatype = metatype;
    obj->firstcolor = obj->color = color;
    obj->height = height;
    obj->radius = radius;
    obj->id = 1000 + gid++;
    Save(obj);
    return obj;
}
// wireframe cuboid

ObjectDB* Scene::CreateWireframeCuboid(std::string name,
        std::string metatype,
        Eigen::Affine3d pose,
        double depth,
        double width,
        double height,
        std::string color, //rviz_visual_tools::colors color,
        const std::string &ns) {
    ObjectDB* obj = new ObjectDB();
    obj->name = name;
    obj->pose = pose;
    obj->metatype = metatype;
    obj->firstcolor = obj->color = color;
    obj->height = height;
    obj->depth = depth;
    obj->width = width;
    obj->id = 1000 + gid++;
    Save(obj);
    return obj;
}

void Scene::Save(ObjectDB * obj) {
    objects.push_back(obj);
}

ObjectDB * Scene::Find(std::size_t id) {
    std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
            boost::bind(&ObjectDB::id, _1) == id);

    if (it != objects.end())
        return *it;

    return NULL;
}

Eigen::Affine3d& Scene::FindPose(std::string name) {
    std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
            boost::bind(&ObjectDB::name, _1) == name);
    if (it != objects.end())
        return (*it)->pose;
    return dummy->pose;
}

ObjectDB * Scene::Find(std::string name) {
    std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
            boost::bind(&ObjectDB::name, _1) == name);
    if (it != objects.end())
        return *it;
    return NULL;
}

std::string Scene::DumpDB() {
    std::stringstream str;
    for (size_t i = 0; i < objects.size(); i++)
        str << objects[i]->name << " " << objects[i]->color << "\n";
    return str.str();

}

Scene::Scene() {

}

void Scene::ClearScene() {
    assert(visual_tools != NULL);
    visual_tools->deleteAllMarkers();
    gid = 1;
    objects.clear(); // leak?
}

void Scene::InitScene() {
    visual_tools = boost::shared_ptr<RvizVisualTools>(new RvizVisualTools("world", "/visualization_marker_array"));
    //visual_tools = boost::shared_ptr<SonOfRvizVisualTools>(new SonOfRvizVisualTools("base_link"));
    visual_tools->enableBatchPublishing(false);

    Globals.Sleep(10000);

    ClearScene();

    // Setup up triangle_marker_ for wall drawing
    triangle_marker_.header.frame_id = "world";

    triangle_marker_.ns = "Triangle";
    triangle_marker_.action = visualization_msgs::Marker::ADD;
    triangle_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    triangle_marker_.lifetime = ros::Duration(0.0);

    // Load Cylinder ----------------------------------------------------
    cylinder_marker_.header.frame_id = "world";
    cylinder_marker_.ns = "Cylinder";
    cylinder_marker_.action = visualization_msgs::Marker::ADD;
    cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker_.lifetime = ros::Duration(0.0);
    cylinder_marker_.id = 1;
}
void Scene::BuildScene() {


#if WALLS
    Save(obj = new ObjectDB(
            "rightwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0.501, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
    ObjectDB::gid++;

    Save(obj = new ObjectDB(
            "backwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.0, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.501, -1.0, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            );
    ObjectDB::gid++;
#endif 
#if 0
    Save(obj = new ObjectDB(
            "leftwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -0.5, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, -0.501, 0.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            ));
    Save(obj = new ObjectDB(
            "backwall", "wall",
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, -1.0, 1.0),
            Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.01, 1.0),
            rviz_visual_tools::TRANSLUCENT_DARK)
            ));

#endif

}

void Scene::NewScene() {

}

// Initialize Eigen::Affine3d http://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix

void Scene::UpdateScene(std::string objname, Eigen::Affine3d pose, std::string color) {
//        rviz_visual_tools::colors color) {
    ObjectDB * obj = Find(objname);
    if (obj == NULL)
        throw std::runtime_error("Gak UpdateScene!");
    obj->color = color;
    obj->pose = pose;
    DrawObject(obj);
    visual_tools->triggerBatchPublish();
}

void Scene::DeleteObject(std::string objname) {
    ObjectDB * obj = Find(objname);
    if (obj == NULL)
        throw std::runtime_error("Gak DeleteObject!");
    cylinder_marker_.action = visualization_msgs::Marker::DELETE;
    DrawObject(obj);
    visual_tools->triggerBatchPublish();
    cylinder_marker_.action = visualization_msgs::Marker::ADD;
}

void Scene::ChangeColor(std::string objname, std::string color) { //rviz_visual_tools::colors color) {
    ObjectDB * obj = Find(objname);
    if (obj == NULL)
        throw std::runtime_error("Gak ChangeColor!");
    obj->color = color;
    cylinder_marker_.action = visualization_msgs::Marker::MODIFY;
    DrawObject(obj);
    visual_tools->triggerBatchPublish();

    cylinder_marker_.action = visualization_msgs::Marker::ADD;

}

bool Scene::DrawObject(ObjectDB *obj) {
    bool b;
    std::string type = ObjectDB::_typemapping[obj->metatype];
    if (type == "mesh") {
        b = visual_tools->publishMesh(obj->pose,
                obj->filepath,
                MARKERCOLOR(obj->color),
                obj->scale,
                type,
                obj->id);
    } else if (type == "cuboid") {
        b = visual_tools->publishCuboid(obj->pose.translation(),
                obj->adjacentpose.translation(),
                MARKERCOLOR(obj->color)); // Eigen::Vector3d(0.0, 0.5, 1.0), 
    } else if (type == "Cylinder") {
        b = publishCylinder(obj->pose,
                obj->color,
                obj-> height,
                obj-> radius,
                obj->id);

    } else if (type == "WireframeCuboid") {
        b=visual_tools->publishWireframeRectangle(obj->pose, 
                obj-> depth, 
                obj->width, 
                MARKERCOLOR(obj->color),
                rviz_visual_tools::REGULAR);  
#if 0  
        b = visual_tools->publishWireframeCuboid(obj->pose,
                obj-> depth,
                obj->width,
                obj-> height,
                MARKERCOLOR(obj->color),
                type.c_str(),
                obj->id);
#endif
    }
     else if (type == "Mark") {
        b = visual_tools->publishSphere(obj->pose,
                 MARKERCOLOR(obj->color),
                 rviz_visual_tools::LARGE,
                 "Sphere", 
                obj->id);
        
    }


    visual_tools->triggerBatchPublish();
    // BOOST_ASSERT_MSG(b == 0, "Failed to publish object");
    LOG_DEBUG << "Draw " << obj->name << "="<< RCS::DumpEigenPose(obj->pose).c_str();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::Duration(0.05).sleep(); // sleep 50 milliseconds
    return b;
}

void Scene::DrawScene() {
    for (size_t i = 0; i < objects.size(); i++) {
        ObjectDB *obj = objects[i];
        bool bFlag = false;
        //while(!bFlag) 
        {
            bFlag = DrawObject(obj);
        }
    }

}

bool Scene::publishCylinder(Eigen::Affine3d pose,
        std::string _color, //rviz_visual_tools::colors color,
        double height,
        double radius,
        size_t &id) {
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
    bool bWorked;
    rviz_visual_tools::colors color = MARKERCOLOR(_color);
    // Set the timestamp
    cylinder_marker_.header.stamp = ros::Time::now();
    cylinder_marker_.ns = "Cylinder";
    //id=cylinder_marker_.id;
    cylinder_marker_.id = id; // ++;

    // Set the pose
    cylinder_marker_.pose = visual_tools->convertPose(pose);

    // Set marker size
    cylinder_marker_.scale.x = radius;
    cylinder_marker_.scale.y = radius;
    cylinder_marker_.scale.z = height;

    // Set marker color
    cylinder_marker_.color = visual_tools->getColor(color);

    // Helper for publishing rviz markers
    bWorked = visual_tools->publishMarker(cylinder_marker_);
    return bWorked;
}