
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
#include "Debug.h"
#include "Config.h"
#include "Shape.h"
using namespace Conversion;

#ifdef _CONFETTI_HEADER_
CMath Math;
std::vector<Piece*> Piece::pieces;
int Piece::n = 0;
#endif

std_msgs::ColorRGBA SceneObject::dummyColor;
SceneObject SceneObject::nullref;
boost::shared_ptr<Scene> pScene = boost::shared_ptr<Scene>(new Scene());
std::map<std::string, rgba> Scene::_color_index=
        map_list_of("RED", rgba(.8, .1, .1))
    ("GREEN",rgba(.1, .8, .1))
    ("GREY",rgba(.9, .9, .9))
    ("DARK_GREY",rgba(.6, .6, .6))
    ("WHITE",rgba(1.0, 1.0, 1.0))
    ("ORANGE",rgba(1.0, 0.5, 0.0))
    ("TRANSLUCENT_LIGHT",rgba(.1, .1, .1, .1))
    ("TRANSLUCENT",rgba(.1, .1, .1, .25))
    ("TRANSLUCENT_DARK",rgba(.1, .1, .1, .5))
    ("BLACK",rgba(0.0, 0.0, 0.0))
    ("YELLOW",rgba(1.0, 1.0, 0.0))
    ("BROWN",rgba(0.597, 0.296, 0.0))
    ("PINK",rgba(1.0, 0.4, 1.0))
    ("LIME_GREEN",rgba(0.6, 1.0, 0.2))
    ("CLEAR",rgba(1.0, 1.0, 1.0, 0.0))
    ("PURPLE",rgba(0.597, 0.0, 0.597))
    ("CYAN",rgba(0.0, 1.0, 1.0))
    ("MAGENTA",rgba(1.0, 0.0, 1.0))
    ("BLUE",rgba(1.0, 1.0, 0.0));
            
std::map<std::string, std::string> SceneObject::_typemapping =
        map_list_of("gear", "mesh")
("mesh", "mesh")
("gearholder", "mesh")
("table", "table")
("wall", "cuboid")
("Checkerboard", "cuboid")
("Cylinder", "Cylinder")
("trayoutline", "WireframeCuboid")
("marker", "Mark")
("confetti", "confetti")
;

std::size_t Scene::gid = 1;
std::string SceneObject::DumpObject(SceneObject& obj){
    std::stringstream s;
    s << "Object=" << obj.name << " Type = " << obj.metatype << "\n";
    return s.str();
}
Scene::Scene()  {
    global_scale_ = 1.0;
 }

std_msgs::ColorRGBA Scene::GetColor(std::string name) {
    // unordered map
    for (std::map<std::string, rgba>::iterator it = _color_index.begin(); it != _color_index.end(); it++) {
        if ((*it).first == name)
            return rgba().SetColorRGBA((*it).second);
    }
    return rgba().SetColorRGBA(rgba(1.0, 1.0, 0.0));
}

std::string Scene::GetColorName(std_msgs::ColorRGBA c) {
    // unordered map
    rgba color(c);
    for (std::map<std::string, rgba>::iterator it = _color_index.begin(); it != _color_index.end(); it++) {
        if (rgba().eq((*it).second, color))
            return (*it).first;
    }
    return "BLUE";
}

void Scene::ClearScene() {
    visualization_msgs::Marker reset_marker_;
    reset_marker_.header.frame_id = frameid;
    reset_marker_.header.stamp = ros::Time();
    reset_marker_.ns = "deleteAllMarkers"; // helps during debugging
    reset_marker_.action = 3;
    marker_pub.publish(reset_marker_);
    ros::spinOnce();
    ros::spinOnce();
    gid = 1;
    objects.clear(); // leak if pointers?  not a leak if using references
}

void Scene::InitScene(ros::NodeHandle & nh, std::string base_frame_) {
    frameid = base_frame_;
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    Globals.Sleep(10000); // waits for rviz to finish loading
    ClearScene();

    generic_marker_.header.frame_id = frameid;
    generic_marker_.action = visualization_msgs::Marker::ADD;
    generic_marker_.lifetime = ros::Duration(0.0);
}

void Scene::SetBaseFrame(std::string base_frame_) {
    frameid = base_frame_;
    ClearScene();
    DrawScene();
}

void Scene::NewScene() {
    ClearScene();
    DrawScene();
}

SceneObject & Scene::CreateMesh(
        std::string name,
        std::string metatype,
        std::size_t id,
        tf::Pose pose,
        std::string filepath,
        std_msgs::ColorRGBA color,
        double scale) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = metatype;
    obj.filepath = filepath;
    obj.scale = scale;
    obj.firstcolor = obj.rawcolor = color;

    if (id == 0)
        obj.id = gid++;
    else obj.id = id;
     return obj;
}
// cuboid

SceneObject & Scene::CreateCuboid(
        std::string name,
        std::string metatype,
        tf::Pose pose,
        tf::Pose adjacentpose,
        std_msgs::ColorRGBA color 
        ) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = metatype;
    obj.adjacentpose = adjacentpose;
    obj.firstcolor = obj.rawcolor = color;
    obj.id = gid++;
    return obj;
}
// marker

SceneObject & Scene::CreateMarker(std::string name,
        tf::Pose pose,
        std_msgs::ColorRGBA color) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = "marker";
    obj.id = gid++;
    obj.firstcolor = obj.rawcolor = color;
    return obj;
}
// cylinder

SceneObject & Scene::CreateCylinder(std::string name,
        std::string metatype,
        tf::Pose pose,
        std_msgs::ColorRGBA color, 
        double height,
        double radius,
        const std::string &ns) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = metatype;
    obj.height = height;
    obj.radius = radius;
    obj.id = gid++;
    obj.firstcolor = obj.rawcolor = color;
    return obj;
}


// Line

SceneObject & Scene::CreateLine(std::string name,
        std::string metatype,
        tf::Vector3 point1,
        tf::Vector3 point2,
        std_msgs::ColorRGBA color,
        double radius) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.metatype = metatype;
    obj.pose = Convert<tf::Vector3, tf::Pose>(point1);
    obj.adjacentpose = Convert<tf::Vector3, tf::Pose>(point2);
    obj.rawcolor = color;
    obj.radius = radius;
    obj.id = gid++;
    return obj;
}

// wireframe cuboid

SceneObject &Scene::CreateWireframeCuboid(std::string name,
        std::string metatype,
        tf::Pose pose,
        double depth,
        double width,
        double height,
        std_msgs::ColorRGBA color,
        const std::string &ns) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = metatype;
    obj.height = height;
    obj.depth = depth;
    obj.width = width;
    obj.id = gid++;
    obj.firstcolor = obj.rawcolor = color;
    return obj;
}


SceneObject & Scene::Find(std::size_t id) {
    std::vector<SceneObject>::iterator it = std::find_if(objects.begin(), objects.end(),
            boost::bind(&SceneObject::id, _1) == id);

    if (it != objects.end())
        return *it;

    return SceneObject::nullref;
}

tf::Pose& Scene::FindPose(std::string name) {
     for (size_t i = 0; i < objects.size(); i++) {
        if (objects[i].name == name)
            return objects[i].pose;
    }
        
    return SceneObject::nullref.pose;
}

SceneObject & Scene::Find(std::string name) {
    for(size_t i=0; i< objects.size(); i++){
        if(objects[i].name == name)
            return objects[i];
    }
    return SceneObject::nullref;
}

bool Scene::CreateWall(std::string name,
        std_msgs::ColorRGBA rgbacolor,
        std::string frameid,
        tf::Vector3 v1,
        tf::Vector3 v2) {
    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.metatype = "wall";
    obj.rawcolor = rgbacolor;
    obj.pose = Convert<tf::Vector3, tf::Pose>(v1);
    obj.adjacentpose = Convert<tf::Vector3, tf::Pose>(v2);
    return true;
}

bool Scene::CreateTable(std::string name,
        std_msgs::ColorRGBA rgbacolor,
        std::string frameid,
        double table_width,
        double table_depth,
        double table_height,
        tf::Pose pose) {

    //rvizMarker->Scale(length, width, height);
    //rvizMarker->SetShape("cube");

    objects.push_back(SceneObject());
    SceneObject & obj(objects.back());
    obj.name = name;
    obj.pose = pose;
    obj.metatype = "table";
    obj.rawcolor = rgbacolor;
    obj.width = table_width;
    obj.depth = table_depth;
    obj.height = table_height;
    obj.id = gid++;
    for (size_t i = 0; i < 4; i++)
        obj.ids.push_back(gid++);

    return true;
}

std::string Scene::DumpDB() {
    std::stringstream str;
    for (size_t i = 0; i < objects.size(); i++)
        str << objects[i].name << " " << objects[i].id << "\n";
    return str.str();

}

#if WALLS
Save(obj = new SceneObject(
        "rightwall", "wall",
        Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, 0.5, 1.0),
        Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, 0.501, 0.0),
        rviz_visual_tools::TRANSLUCENT_DARK)
        );
SceneObject::gid++;

Save(obj = new SceneObject(
        "backwall", "wall",
        Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.0, 1.0),
        Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.501, -1.0, 0.0),
        rviz_visual_tools::TRANSLUCENT_DARK)
        );
SceneObject::gid++;
#endif 
#if 0
Save(obj = new SceneObject(
        "leftwall", "wall",
        Eigen::Affine3d::Identity() * Eigen::Translation3d(0.0, -0.5, 1.0),
        Eigen::Affine3d::Identity() * Eigen::Translation3d(1.0, -0.501, 0.0),
        rviz_visual_tools::TRANSLUCENT_DARK)
        ));
Save(obj = new SceneObject(
        "backwall", "wall",
        Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, -1.0, 1.0),
        Eigen::Affine3d::Identity() * Eigen::Translation3d(-0.5, 1.01, 1.0),
        rviz_visual_tools::TRANSLUCENT_DARK)
        ));

#endif

void Scene::UpdateScene(SceneObject & obj) {
     DrawObject(obj);
}

void Scene::UpdateScene(std::string objname, tf::Pose pose, std_msgs::ColorRGBA color) {
    SceneObject & obj = Find(objname);
    if (IsNull(obj))
        throw std::runtime_error("Gak UpdateScene!");
    obj.rawcolor = color;
    obj.pose = pose;
    DrawObject(obj);
}

void Scene::DeleteObject(std::string objname) {
    SceneObject & obj = Find(objname);
    if (IsNull(obj))
        throw std::runtime_error("Gak DeleteObject!");
    generic_marker_.action = visualization_msgs::Marker::DELETE;
    DrawObject(obj);
    generic_marker_.action = visualization_msgs::Marker::ADD;
}

void Scene::ChangeColor(std::string objname, std_msgs::ColorRGBA color) { //rviz_visual_tools::colors color) {
    SceneObject & obj = Find(objname);
    if (IsNull(obj))
        throw std::runtime_error("Gak ChangeColor!");
    obj.rawcolor = color;
    generic_marker_.action = visualization_msgs::Marker::MODIFY;
    DrawObject(obj);
    generic_marker_.action = visualization_msgs::Marker::ADD;

}

bool Scene::DrawObject(SceneObject &obj) {
    bool b;
    std::string type = SceneObject::_typemapping[obj.metatype];
    if (type == "mesh") {
        publishMesh(obj.pose,
                obj.filepath,
                obj.rawcolor,
                obj.scale,
                obj.id);
    } else if (type == "cuboid") {
        std_msgs::ColorRGBA color = obj.rawcolor;
        publishCuboid(obj.pose,
                obj.adjacentpose,
                color,
                obj.id);
    } else if (type == "table") {
        std_msgs::ColorRGBA color = obj.rawcolor;
        tf::Pose adjpose = obj.pose;
        adjpose.getOrigin().setZ( adjpose.getOrigin().z() - obj.height / 2.0);
        // x=depth, y=width, z = height
        publishCuboid(adjpose,
                obj. depth, obj. width, obj. height,
                color,
                obj.id);

         double x = obj.pose.getOrigin().x();
        double y = obj.pose.getOrigin().y();
        double z = obj.pose.getOrigin().z();
        double minx = x - obj.depth;
        double maxx = x + obj.depth;
        double miny = y - obj.width;
        double maxy = y + obj.width;

        // legs
        double legradius;
        double legwidth;
        double length = obj.width;
        double width = obj.depth;
        double height = obj.height;
        legradius = legwidth = 0.05;

        tf::Pose leg1, leg2, leg3, leg4;
        leg1.setOrigin(tf::Vector3(x + (length / 2.0) - legradius, y - (width / 2.0) + legradius, (z - height) / 2.0));
        leg1.setRotation(tf::QIdentity());
        leg2.setOrigin(tf::Vector3(x - (length / 2.0) + legradius, y + (width / 2.0) - legradius, (z - height) / 2.0));
        leg3.setRotation(tf::QIdentity());
        leg3.setOrigin(tf::Vector3(x + (length / 2.0) - legradius, y + (width / 2.0) - legradius, (z - height) / 2.0));
        leg3.setRotation(tf::QIdentity());
        leg4.setOrigin(tf::Vector3(x - (length / 2.0) + legradius, y - (width / 2.0) + legradius, (z - height) / 2.0));
        leg4.setRotation(tf::QIdentity());

        height = obj.pose.getOrigin().z() - obj.height;

        publishCylinder(leg1, color, height, legradius, obj.ids[0]);
        publishCylinder(leg2, color, height, legradius, obj.ids[1]);
        publishCylinder(leg3, color, height, legradius, obj.ids[2]);
        publishCylinder(leg4, color, height, legradius, obj.ids[3]);
        ros::Duration(0.05).sleep();

    } else if (type == "Cylinder") {
        std_msgs::ColorRGBA color = obj.rawcolor;
        publishCylinder(obj.pose,
                color,
                obj.height,
                obj.radius,
                obj.id);

    } else if (type == "WireframeCuboid") {
        publishWireframeRectangle(obj.pose,
                obj.depth,
                obj.width,
                obj.rawcolor, // MARKERCOLOR("YELLOW"),
                0.01,
                obj.id);
    } else if (type == "Mark") {
        publishSphere(obj.pose,
                obj.rawcolor,
                0.025,
                obj.id);
    } else if (type == "confetti") {
        publishLine(
                Convert<tf::Pose, tf::Vector3>(obj.pose),
                Convert<tf::Pose, tf::Vector3>(obj.adjacentpose),
                obj.rawcolor,
                obj.radius,
                obj.id);
    }

    // BOOST_ASSERT_MSG(b == 0, "Failed to publish object");
#if defined(LogScene)
    ofsScene << "Draw " << obj.name << "=" << RCS::DumpPose(obj.pose).c_str() << "\n" << std::flush;
#endif
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::Duration(0.05).sleep(); // sleep 50 milliseconds
    return b;
}

void Scene::MakeConfetti() {

    for (size_t i = 0; i < 20; i++) {

        double x = ((double) i)*.01;
        CreateLine("confetti1", "confetti",
                tf::Vector3(x, 0.0, 1.0), tf::Vector3(x, 0.0, 1.005),
                rgba(Random(0.0, 128.0),
                Random(0.0, 128.0),
                Random(0.0, 128.0),
                1.0).GetColorRGBA(),
                .005);

    }
}

void Scene::DrawScene() {
    for (size_t i = 0; i < objects.size(); i++) {
        SceneObject &obj = objects[i];
        bool bFlag = false;
        //while(!bFlag) 
        {
            bFlag = DrawObject(obj);
        }
    }

}

void Scene::publishCylinder(tf::Pose pose,
        std_msgs::ColorRGBA _color,
        double height,
        double radius,
        size_t id) {
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
    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
 //   generic_marker_.action = visualization_msgs::Marker::ADD;
    generic_marker_.type = visualization_msgs::Marker::CYLINDER;
    generic_marker_.ns = "Cylinder";
    generic_marker_.id = id;

    // Set the pose
    generic_marker_.pose = Convert<tf::Pose, geometry_msgs::Pose >(pose);


    // Set marker size
    generic_marker_.scale.x = radius;
    generic_marker_.scale.y = radius;
    generic_marker_.scale.z = height;

    // Set marker color
    generic_marker_.color = _color;

    // Helper for publishing rviz markers
    marker_pub.publish(generic_marker_);
 }

void Scene::publishLine(const tf::Vector3 &point1,
        const tf::Vector3 &point2,
        const std_msgs::ColorRGBA &color,
        double radius,
        size_t &id) {

    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    geometry_msgs::Vector3 scale;
    scale.x = radius * global_scale_;
    scale.y = radius * global_scale_;
    scale.z = radius * global_scale_;

    generic_marker_.id = id;
    generic_marker_.ns = "line";
    generic_marker_.color = color;
    generic_marker_.scale = scale;

    generic_marker_.points.clear();
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(point1));
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(point2));

    // Helper for publishing rviz markers
    marker_pub.publish(generic_marker_); // line_strip_marker_);
}

/**
 * \brief Display a rectangular cuboid
 * \param point1 - x,y,z top corner location of box
 * \param point2 - x,y,z bottom opposite corner location of box
 * \param color - color of marker
 * \param id  id of marker
 * \return true on success
 */
void Scene::publishCuboid(tf::Pose &pose1, tf::Pose &pose2,
        std_msgs::ColorRGBA color,
        size_t id) {
    // Calculate center pose
    tf::Vector3 point1 = Convert<tf::Pose, tf::Vector3>(pose1);
    tf::Vector3 point2 = Convert<tf::Pose, tf::Vector3>(pose2);

    tf::Pose midpoint = tf::Identity();
    midpoint.setOrigin(tf::Vector3((point1.x() - point2.x()) / 2.0 + point2.x(),
            (point1.y() - point2.y()) / 2.0 + point2.y(),
            (point1.z() - point2.z()) / 2.0 + point2.z()));
    // Calculate scale
    double depth = fabs(point1.x() - point2.x());
    double width = fabs(point1.y() - point2.y());
    double height = fabs(point1.z() - point2.z());
    publishCuboid(midpoint, depth, width, height, color, id);
}

void Scene::publishCuboid(tf::Pose &midpoint, double depth, double width, double height,
        std_msgs::ColorRGBA color,
        size_t id) {

    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.ns = "cuboid";
    generic_marker_.type = visualization_msgs::Marker::CUBE;
    generic_marker_.id = id;
    generic_marker_.color = color;

    generic_marker_.pose = Convert<tf::Pose, geometry_msgs::Pose >(midpoint);

    // Calculate scale
    generic_marker_.scale.x = depth; // fabs(point1.x - point2.x);
    generic_marker_.scale.y = width; // fabs(point1.y - point2.y);
    generic_marker_.scale.z = height; // fabs(point1.z - point2.z);

    // Prevent scale from being zero
    if (!generic_marker_.scale.x)
        generic_marker_.scale.x = 0.0001;
    if (!generic_marker_.scale.y)
        generic_marker_.scale.y = 0.0001;
    if (!generic_marker_.scale.z)
        generic_marker_.scale.z = 0.0001; //SMALL_SCALE;

    // Helper for publishing rviz markers
    marker_pub.publish(generic_marker_);
}

void Scene::publishSphere(const tf::Pose &pose, std_msgs::ColorRGBA color, double scale, std::size_t id) {
    // Set the frame ID and timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.ns = "sphere";
    generic_marker_.type = visualization_msgs::Marker::SPHERE;

    generic_marker_.id = id;
    generic_marker_.pose = Convert<tf::Pose, geometry_msgs::Pose >(pose);
    generic_marker_.color = color;
    generic_marker_.scale = Convert<double, geometry_msgs::Vector3 >(scale * global_scale_); //geometry_msgs::Vector3 scale
    marker_pub.publish(generic_marker_);
}

void Scene::publishMesh(const tf::Pose &pose, const std::string &file_name, std_msgs::ColorRGBA color,
        double scale, std::size_t id) {
    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.ns = "mesh";
    generic_marker_.id = id;
    generic_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;

    // Set the mesh
    generic_marker_.mesh_resource = file_name;
    generic_marker_.mesh_use_embedded_materials = true;

    // Set the pose
    generic_marker_.pose = Convert<tf::Pose, geometry_msgs::Pose >(pose);

    // Set marker size
    generic_marker_.scale.x = scale;
    generic_marker_.scale.y = scale;
    generic_marker_.scale.z = scale;

    generic_marker_.color = color;
    marker_pub.publish(generic_marker_);
}

void Scene::publishText(const tf::Pose &pose, const std::string &text, std_msgs::ColorRGBA color,
        const tf::Vector3 scale, std::size_t id) {
    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.ns = "text";
    generic_marker_.id = id;
    generic_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    generic_marker_.header.frame_id = frameid;
    generic_marker_.text = text;
    generic_marker_.pose = Convert<tf::Pose, geometry_msgs::Pose >(pose);
    generic_marker_.color = color;
    generic_marker_.scale = Convert<tf::Vector3, geometry_msgs::Vector3 >(scale);
    

    marker_pub.publish(generic_marker_);
}

void Scene::publishWireframeRectangle(const tf::Pose &pose, double height, double width, std_msgs::ColorRGBA color,
        double scale, std::size_t id) {
    // Set the timestamp
    generic_marker_.header.stamp = ros::Time::now();
    generic_marker_.ns = "Wireframe Rectangle";
    generic_marker_.id = id;
    generic_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;


    // Extract 8 cuboid vertices
    tf::Vector3 p1(-width / 2.0, -height / 2.0, 0.0);
    tf::Vector3 p2(-width / 2.0, height / 2.0, 0.0);
    tf::Vector3 p3(width / 2.0, height / 2.0, 0.0);
    tf::Vector3 p4(width / 2.0, -height / 2.0, 0.0);

    p1 = pose * p1;
    p2 = pose * p2;
    p3 = pose * p3;
    p4 = pose * p4;

    // Setup marker

    std_msgs::ColorRGBA this_color = color;
    generic_marker_.scale = Convert<double, geometry_msgs::Vector3 >(scale);
    generic_marker_.color = this_color;
    generic_marker_.points.clear();
    generic_marker_.colors.clear();

    // Add each point pair to the line message
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p1));
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p2));
    generic_marker_.colors.push_back(this_color);
    generic_marker_.colors.push_back(this_color);

    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p2));
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p3));
    generic_marker_.colors.push_back(this_color);
    generic_marker_.colors.push_back(this_color);

    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p3));
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p4));
    generic_marker_.colors.push_back(this_color);
    generic_marker_.colors.push_back(this_color);

    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p4));
    generic_marker_.points.push_back(Convert<tf::Vector3, geometry_msgs::Point>(p1));
    generic_marker_.colors.push_back(this_color);
    generic_marker_.colors.push_back(this_color);

    marker_pub.publish(generic_marker_);
}


/**

  Evenly allocate RGB colors around HSL color wheel

  @param[out] v_picked_cols  a vector of colors in RGB format
  @param[in]  count   number of colors required
  @param[in]  bright  0 is all black, 100 is all white, defaults to 50

  based on Fig 3 of http://epub.wu-wien.ac.at/dyn/virlib/wp/eng/mediate/epub-wu-01_c87.pdf?ID=epub-wu-01_c87

 */

//void cColorPicker::Pick( vector<DWORD>&v_picked_cols, int count, int bright )

void cColorPicker::Pick(vector<rgba>&v_picked_cols, int count, int bright) {
    v_picked_cols.clear();
    for (int k_hue = 0; k_hue < 360; k_hue += 360 / count)
        v_picked_cols.push_back(HSL2RGB(k_hue, 100, bright));
}
/**

  Convert HSL to RGB

  based on http://www.codeguru.com/code/legacy/gdi/colorapp_src.zip
  from: http://stackoverflow.com/questions/180/function-for-creating-color-wheels

 */

//DWORD cColorPicker::HSL2RGB( int h, int s, int l )

rgba cColorPicker::HSL2RGB(int h, int s, int l) {
    unsigned long ret = 0;
    UCHAR r, g, b;

    float saturation = s / 100.0f;
    float luminance = l / 100.f;
    float hue = (float) h;

    if (saturation == 0.0) {
        r = g = b = UCHAR(luminance * 255.0);
    } else {
        float rm1, rm2;

        if (luminance <= 0.5f) rm2 = luminance + luminance * saturation;
        else rm2 = luminance + saturation - luminance * saturation;
        rm1 = 2.0f * luminance - rm2;
        r = ToRGB1(rm1, rm2, hue + 120.0f);
        g = ToRGB1(rm1, rm2, hue);
        b = ToRGB1(rm1, rm2, hue - 120.0f);
    }

    //ret = ((DWORD)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)));

    return rgba(((double) r) / 255., ((double) g) / 255., ((double) b) / 255.);
}

unsigned char cColorPicker::ToRGB1(float rm1, float rm2, float rh) {
    if (rh > 360.0f) rh -= 360.0f;
    else if (rh < 0.0f) rh += 360.0f;

    if (rh < 60.0f) rm1 = rm1 + (rm2 - rm1) * rh / 60.0f;
    else if (rh < 180.0f) rm1 = rm2;
    else if (rh < 240.0f) rm1 = rm1 + (rm2 - rm1) * (240.0f - rh) / 60.0f;

    return static_cast<unsigned char> (rm1 * 255);
}

