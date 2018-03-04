
#ifndef Shape_Model_H
#define Shape_Model_H

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <list>
#include <cstring>
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>
#include "NIST/Boost.h"


namespace pt = boost::property_tree;



/**
http://zenol.fr/blog/boost-property-tree/en.html
 */
namespace ShapeModel {


    //const double convfactor=0.001;
    const double convfactor = 1.0;

    struct Pose {
        double translation[3];

        union {
            double orientation[4];
            double rpy[3];
        };
    };

    struct Circle {
        double diameter;

        //        Circle() {
        //            diameter = 0.0;
        //        }
    };
    // Rectangle is used by Windows!

    struct Rect {
        double height;
        double width;

        //        Rect() {
        //            height = width = 0.0;
        //        }
    };

    struct Cube {
        double depth, width, height;

        //        Cube() {
        //            depth = height = width = 0.0;
        //        }
    };

    struct Point {
        Pose point;
    };

    struct Line {
        Pose pt1;
        Pose pt2;
    };

    struct Polygon {

        /**
        Alternative Solution to Union:
        You can use boost::variant */
        union {
            //std::vector<Pose> vpts; 
            Pose pts[10];
        };
    };

    struct Mesh {
        std::string meshfile; // this can't be used in union
        double scale;
    };

    struct Shape {
        std::string name;
        std::string propname;
        std::string type;
        std::string subtype; // wireframe, solid
        std::string metatype;
        std::string color;

        std::string meshfile;
        double scale;

        Pose centroid;
        double x, y, z;

        union {
            Cube cube;
            Rect rect;
            Circle circle;
            Point point;
            Line line;
            Polygon polygon;
        };
        std::list< Shape > contains;
    };

    struct Instance {
        std::string name;
        std::string propname;
        std::string type;
        std::string metatype;
        std::string subtype;
        double rotation;
        double x, y, z;
        std::string color;
        std::map<std::string, std::string> properties;
        Shape shape;
        bool operator<(const Instance& b) const {
            return strcasecmp(this->name.c_str(), b.name.c_str()) < 0;
        }
    };

    inline std::string DumpInstance(Instance & instance) {
        std::stringstream s;
        s << "Name = " << instance.name.c_str() << ":Type = " << instance.type << ":Metaype = " << instance.metatype;
        s << ":Rotation = " << instance.rotation << ":X= " << instance.x << ":Y= " << instance.y << std::endl;
        return s.str();
    }

    struct Shapes {
        void ParseJsonFile(std::string filename);

        std::vector<tf::Pose> TypeInstanceLocations(std::string sku);
        std::vector<boost::shared_ptr<Instance> > TypeInstances(std::string);
        boost::shared_ptr<Instance> NamedInstance(std::string);
        
        std::vector<tf::Pose> CopySkuHolderLocations(
                std::string sku = "sku_part_small_gear",
                std::string skuholder = "sku_part_small_gear_vessel");
        tf::Pose GetInstancePose(boost::shared_ptr<Instance> instance);
        Shape & GetInstanceShape(boost::shared_ptr<Instance> instance);
        tf::Vector3 GetInstancePosition(boost::shared_ptr<Instance> instance);
        tf::Quaternion GetInstanceRotation(boost::shared_ptr<Instance> instance);
        tf::Pose GetChildPose(std::string childname);
        
        template <typename T>
        T GetInstanceValue(boost::shared_ptr<Instance> instance, std::string child) {
            return root.get<T>("instances." + instance->name + "." + child, T());
        }
        template <typename T>
        T GetChildValue(std::string path) {
            return root.get<T>(path, T());
        }
        template <typename T>
        void PutChildValue(std::string path, T val) {
            root.put<T>(boost::property_tree::ptree::path_type{path.c_str(), '.'}, val);
            // root.put<T>(path, val);
        }
        template <typename T>
        std::vector<T> GetChildValues(std::string childpath) {
            return GetTypes<T>(root, childpath);
        }
        std::vector<std::string> GetShapeChildrenNames(std::string partname, std::string child) {
            std::vector<std::string> names;
            pt::ptree &c = root.get_child("parts." + partname + "." + child);
            for (pt::ptree::const_iterator it = c.begin(); it != c.end(); it++) {
                std::string name = (*it).first.data();
                names.push_back("parts." + partname + "." + child + "." + name);
            }
            return names;
        }
         std::vector<std::string> GetChildrenNames(std::string childpath) {
            std::vector<std::string> names;
            pt::ptree &c = root.get_child(childpath);
            for (pt::ptree::const_iterator it = c.begin(); it != c.end(); it++) {
                std::string name = (*it).first.data();
                names.push_back( name);
            }
            return names;
         }

#if 1
        // http://stackoverflow.com/questions/8154107/how-do-i-merge-update-a-boostproperty-treeptree
         void traverse_recursive(const boost::property_tree::ptree &parent, 
        const boost::property_tree::ptree::path_type &childPath, 
        const boost::property_tree::ptree &child){
            using boost::property_tree::ptree;

            merge(parent, childPath, child);
            for (ptree::const_iterator it = child.begin(); it != child.end(); ++it) {
                ptree::path_type curPath = childPath / ptree::path_type(it->first);
                traverse_recursive(parent, curPath, it->second); 
            }
        }

         void traverse(const pt::ptree &parent) { 
            traverse_recursive(parent, "", parent); 
        }

        void update_ptree(const pt::ptree & pt) {
            using namespace boost;
            traverse(root); 
        }
#endif
        pt::ptree & ptree(){ return root; }
    protected:

        void merge(const pt::ptree &parent, 
                const pt::ptree::path_type &childPath, 
                const pt::ptree & child) {
            root.put(childPath, child.data());
        }
        pt::ptree root;
        Shape ParseShape(std::string prefix, std::string name, pt::ptree & root);
//        std::map<Instance, std::vector<Instance> > instances;
        std::map<std::string, std::vector< boost::shared_ptr<Instance> > > metatypeinstances;
        std::map<std::string, std::vector< boost::shared_ptr<Instance> > > typeinstances;
        std::map<std::string, std::vector< boost::shared_ptr<Instance> > > namedinstances;
        std::map<std::string, Shape > parts;
    };

};
#endif
