

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
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>
#include <boost/bind.hpp>
#include <algorithm>
#include <map>

#include "Globals.h"
#include "RCS.h"
#include "Shape.h"
#include "Conversions.h"

/**
 * \brief Class wrapper for rbga color scheme.
 * RGB color definition and alpha A are all represented as 0..1 doubles.
 */

struct rgba {

    /**
     * \brief Empty constructor.
     */
    rgba() {
    }

    /**
     * \brief RBGA constructor, where Colors RGB and alpha A are all represented as 0..1 doubles.
     */
    rgba(int r, int g, int b) {
        SetColorRGBA((double) r / 255.0, (double) g / 255.0, (double) b / 255.0, 1.0);
    }

    /**
     * \brief RBGA constructor, where Colors RGB and alpha A are all represented as 0..1 doubles.
     */
    rgba(double r_, double g_, double b_, double a_ = 1.0) : r(r_), g(g_), b(b_), a(a_) {
    }

    rgba(std_msgs::ColorRGBA c) : r(c.r), g(c.g), b(c.b), a(c.a) {
    }

    /**
     * \brief Equal method for determining equality of color coding instances.
     */
    bool eq(const rgba & c, const rgba & d) {
        return (d.a == c.a) && (d.r == c.r) && (d.g == c.g) && (d.b == c.b);
    }
    double r, g, b, a; /**< RBGA valudes represented as 0..1 doubles */

    std_msgs::ColorRGBA SetColorRGBA(double r, double g, double b, double a) {
        return SetColorRGBA(rgba(r, g, b, a));
    }

    std_msgs::ColorRGBA GetColorRGBA() {
        std_msgs::ColorRGBA val;
        val.r = r;
        val.g = g;
        val.b = b;
        val.a = a;
        return val;
    }

    std_msgs::ColorRGBA SetColorRGBA(rgba color) {
        std_msgs::ColorRGBA val;
        val.r = color.r;
        val.g = color.g;
        val.b = color.b;
        val.a = color.a;
        return val;
    }
};

/**
 * The SceneObject is a class to represent various RVIZ markers in the "scene". 
 * There can be a number of objects in a scene, Depending on the instance, different 
 * parameters from the class are used. The class definition is merely a union of all
 * possible parameters. Depending on the SceneObject type a different set of parameters 
 * will be used.
 * 
 * */
class SceneObject {
public:
    static SceneObject nullref;

    SceneObject() {
        rawcolor = dummyColor;
    }

    SceneObject(
            std::string name,
            std::string metatype,
            std::size_t id) {
        rawcolor = dummyColor;

        this->name = name;
        this->metatype = metatype;
        this->id = id;
    }
    std::size_t id;
    std::vector<std::size_t> ids; // for multiple markers per object
    std::string name;
    std::string metatype; /**< mesh, cuboid, cylinder */
    std::string subtype; /**< solid, wireframe */
    //std::string color;
    std_msgs::ColorRGBA rawcolor, firstcolor;
    tf::Pose pose;
    tf::Pose gripperoffset;
    double gripperclosewidth; // displacement of gripper closed in meters

    // Mesh definitionss
    std::string filepath; /**< file path of the mesh file (STL) */
    double scale; /**< scale factor of the mesh */

    //std::string firstcolor; // rviz_visual_tools::colors firstcolor;

    tf::Pose adjacentpose;
    tf::Vector3 centroid;
    double depth, width, height, radius;

    // Metatype name mapping into rviz_visual_tools id marker type (e.g. gear into mesh)
    static std::map<std::string, std::string> _typemapping;
    boost::shared_ptr<ShapeModel::Instance> instance;
    static std_msgs::ColorRGBA dummyColor;
    static std::string DumpObject(SceneObject&);

};

/**
 * \brief The scene represents all the objects to be drawn in rviz using rviz_visual_tools. 
 * There can be a number of objects in a scene.You could just use the package rviz_visual_tools,
 * but this helps with some other scene objects. Doubt if its thread safe.
 * 
 * */
struct Scene {
    Scene();
    static std::string GetColorName(std_msgs::ColorRGBA c);
    static std_msgs::ColorRGBA GetColor(std::string name);
    bool DrawObject(SceneObject &obj);
    void ClearScene();
    void NewScene();
    void InitScene(ros::NodeHandle & nh, std::string base_frame_);
    void ChangeColor(std::string objname, std_msgs::ColorRGBA color);
    void UpdateScene(std::string objname, tf::Pose pose, std_msgs::ColorRGBA color);
    void UpdateScene(SceneObject & obj);
    void DrawScene();
    void DeleteObject(std::string objname);


    SceneObject & CreateMesh(
            std::string name,
            std::string metatype,
            std::size_t id,
            tf::Pose pose,
            std::string filepath,
            std_msgs::ColorRGBA color,
            double scale = 1.0);

    // cuboid
    SceneObject & CreateCuboid(
            std::string name,
            std::string metatype,
            tf::Pose pose = tf::Identity(),
            tf::Pose adjacentpose = tf::Identity(),
            std_msgs::ColorRGBA color = GetColor("CLEAR")
            );

    // cylinder
    SceneObject & CreateCylinder(std::string name,
            std::string metatype,
            tf::Pose pose,
            std_msgs::ColorRGBA color,
            double height,
            double radius,
            const std::string &ns);

    // Line
    SceneObject & CreateLine(std::string name,
            std::string metatype,
            tf::Vector3 point1,
            tf::Vector3 point2,
            std_msgs::ColorRGBA color,
            double radius);

    // x cross marker
    SceneObject & CreateMarker(std::string name,
            tf::Pose pose,
            std_msgs::ColorRGBA color);

    // wireframe cuboid
    SceneObject& CreateWireframeCuboid(std::string name,
            std::string metatype,
            tf::Pose pose,
            double depth = 0.005,
            double width = 0.005,
            double height = 0.01,
            std_msgs::ColorRGBA color = GetColor("GREEN"),
            const std::string &ns = "WireframeCuboid");

    bool CreateWall(std::string name,
            std_msgs::ColorRGBA rgbacolor,
            std::string frameid,
            tf::Vector3 v1,
            tf::Vector3 v2);

    bool CreateTable(std::string name,
            std_msgs::ColorRGBA rgbacolor,
            std::string frameid,
            double table_length,
            double table_width,
            double table_height,
            tf::Pose centertablepose);

    //void Save(SceneObject & obj);
    // FIXME: add delete object
    SceneObject & Find(std::size_t id);
    tf::Pose& FindPose(std::string name);
    SceneObject & Find(std::string name);
    std::string DumpDB();

    void SetBaseFrame(std::string base_frame_);

    std::string GetBaseFrame() {
        return frameid;
    }

    void MakeConfetti();

    static bool IsNull(SceneObject & obj) {
        return &obj == &SceneObject::nullref;
    }

    static SceneObject & NullObj() {
        return SceneObject::nullref;
    }
    static std::size_t gid;

private:
    ros::Publisher marker_pub;
    /**
     * \brief Array of matching rviz visual tools color string and corresdponing  rgb and alpha color.
     */
    static std::map<std::string, rgba> _color_index;
    std::string frameid;
    double global_scale_;
    std::vector<SceneObject> objects;
    std::map<std::string, std::vector<SceneObject> > compound_objs;
    visualization_msgs::Marker generic_marker_;

    void publishCylinder(tf::Pose pose,
            std_msgs::ColorRGBA color,
            double radius,
            double height,
            size_t id);
    void publishCuboid(tf::Pose &pose1, tf::Pose &pose2,
            std_msgs::ColorRGBA color,
            size_t id);
    void publishCuboid(tf::Pose &midpoint, double depth, double width, double height,
            std_msgs::ColorRGBA color,
            size_t id);
    void publishLine(const tf::Vector3 &point1,
            const tf::Vector3 &point2,
            const std_msgs::ColorRGBA &color,
            double radius,
            size_t &id);
    void Scene::publishSphere(const tf::Pose &pose,
            std_msgs::ColorRGBA color,
            double scale,
            std::size_t id);
    void publishMesh(const tf::Pose &pose,
            const std::string &file_name,
            std_msgs::ColorRGBA color,
            double scale,
            std::size_t id);
    void publishText(const tf::Pose &pose,
            const std::string &text,
            std_msgs::ColorRGBA color,
            const tf::Vector3 scale,
            std::size_t id);
    void publishWireframeRectangle(const tf::Pose &pose,
            double height,
            double width,
            std_msgs::ColorRGBA color,
            double scale,
            std::size_t id);
};

extern boost::shared_ptr<Scene> pScene;


#include "confetti.h"

struct Piece {
protected:
    int type;
    double d;
    double dx;
    double dy;
    double dz;
    double m;
    double x;
    double y;
    double z;
    SceneObject& obj;
    static int n;
    boost::shared_ptr<Scene> pScene;
public:

    std::vector<std::vector<double> > values;
    double value;
    std_msgs::ColorRGBA color;
    static std::vector<Piece*> pieces;

    Piece(boost::shared_ptr<Scene> _pScene) : pScene(_pScene), obj(SceneObject::nullref) {
        type = Math.floor(Math.random() * 5);
        d = .1;
        dx = 0;
        dy = 0;
        dz = 0;
        m = 0;
        pieces.push_back(this);
    }

    tf::Vector3 compute_dxyz() {
        double a = Math.random() * Math.PI * 2;

        m = 0.0;
        double s;

        switch (type) {
            case 1:
                s = Math.random() * 2;
                break;
            case 2:
                s = 2;
                break;
            case 3:
                s = (Math.PI * 2) - a - Math.random();
                break;
            case 4:
                s = a - Math.random();
                break;
            default:
                s = Math.random() * 2;
                if (Math.random() > .6) {
                    s = 1.5;
                }
        }

        dx = ((s + 4) * Math.sin(a)) / 1000.0;
        dy = ((s + 4) * Math.cos(a) - 2) / 1000.0;
        dz = .005;
        return tf::Vector3(dx, dy, dz);
    }

    void launch(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;

        tf::Vector3 origin(x, y, z);
        tf::Vector3 offset = origin + compute_dxyz();

        obj = pScene->CreateLine(Globals.StrFormat("confetti%d", n++), "confetti",
                origin, offset,
                rgba(Random(0.0, 128.0), Random(0.0, 128.0), Random(0.0, 128.0)).GetColorRGBA(),
                .005);
    }

    void Update() {
        //SceneObject * obj = pScene->Find(StrFormat("confetti%d",i));
        tf::Pose diff(tf::QIdentity(), tf::Vector3(dx, dy, dz));
        obj.pose = obj.pose * diff;
        obj.adjacentpose = obj.adjacentpose * diff;
        pScene->UpdateScene(obj);
    }

};


/**
int _tmain(int argc, _TCHAR* argv[])
{
vector<rgba> myCols;
cColorPicker colpick;
colpick.Pick( myCols, 20 );
for( int k = 0; k < (int)myCols.size(); k++ )
printf("%d: %d %d %d\n", k+1,
( myCols[k] & 0xFF0000 ) >>16,
( myCols[k] & 0xFF00 ) >>8,
( myCols[k] & 0xFF ) );

return 0;
}
 */
typedef unsigned char UCHAR;

class cColorPicker {
public:
    void Pick(std::vector<rgba>&v_picked_cols, int count, int bright = 50);
    //  void Pick( std::vector<DWORD>&v_picked_cols, int count, int bright = 50 );
private:
    rgba HSL2RGB(int h, int s, int v);
    // DWORD HSL2RGB( int h, int s, int v );
    unsigned char ToRGB1(float rm1, float rm2, float rh);
};
