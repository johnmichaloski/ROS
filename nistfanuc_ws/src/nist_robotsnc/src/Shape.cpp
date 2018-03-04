

#include "nist_robotsnc/Shape.h"
#include "nist_robotsnc/NIST/Boost.h"
#include <boost/optional.hpp>
#include "nist_robotsnc/Conversions.h"
using namespace Conversion;

namespace ShapeModel {



    Shape Shapes::ParseShape(std::string prefix, std::string name, pt::ptree &root) {
        Shape shape;
        shape.name = name; 
        shape.propname = prefix + name; 
        shape.metatype = root.get<std::string>(prefix + name + ".metatype");
        shape.type = root.get<std::string>(prefix + name + ".type");
        shape.subtype = root.get<std::string>(prefix + name + ".subtype","none");
        shape.color = root.get<std::string>(prefix + name + ".color","parent");
        if (shape.metatype == "mesh") {
            shape.scale = root.get<double>(prefix + name + ".scale");
            shape.meshfile = root.get<std::string>(prefix + name + ".file");
        }
        if (shape.metatype == "Rectangle") {
            shape.rect.height = root.get<double>(prefix + name + ".height");
            shape.rect.width = root.get<double>(prefix + name + ".width");
        } else if (shape.metatype == "Circle") {
            shape.circle.diameter = root.get<double>(prefix + name + ".diameter");
        } else if (shape.metatype == "Cube") {
            shape.cube.height = root.get<double>(prefix + name + ".height");
            shape.cube.width = root.get<double>(prefix + name + ".width");
            shape.cube.depth = root.get<double>(prefix + name + ".depth");
        }
        boost::optional<std::string> pos = root.get_optional<std::string>(prefix + name + ".position");
        if (pos) {
            std::vector<double> position = GetTypes<double>(root, prefix + name + ".position");
            shape.x = position[0]*convfactor;
            shape.y = position[1]*convfactor;
        }
        return shape;

    }

    void Shapes::ParseJsonFile(std::string filename) {
        try {
            // Load the json file in this ptree
            pt::read_json(filename, root);

            BOOST_FOREACH(pt::ptree::value_type &v, root.get_child("parts")) {
                Shape shape = ParseShape("parts.", v.first.data(), root);
                parts[shape.name] = shape;
                boost::optional<std::string> c = root.get_optional<std::string>("parts." + shape.name + ".contains");
                if (c) {
                    BOOST_FOREACH(pt::ptree::value_type &v3,
                            root.get_child("parts." + shape.name + ".contains"))
                    {
                        std::string name = v3.first.data();
                        Shape cshape = ParseShape("parts." + shape.name + ".contains.", name , root);               
                        shape.contains.push_back(cshape);
                    }
                }
            }

            BOOST_FOREACH(pt::ptree::value_type &v1,
                    root.get_child("instances")) {
                boost::shared_ptr<Instance> instance =boost::shared_ptr<Instance>(new Instance()) ;
                //Instance instance;
                instance->name = v1.first.data();
                instance->propname = "instances." + instance->name;
                instance->type = root.get<std::string>("instances." + instance->name + ".type");
                instance->metatype = root.get<std::string>("instances." + instance->name + ".metatype");
                if (instance->type == "outline") {
                    // FIXME FIXME
 //                   instance.shape = ParseShape("instances." + instance.name, instance.name, root);
                } else {
                    // Check - should exist since this is an instance of a part model
                    if (parts.find(instance->metatype) == parts.end()) {
                        assert(0);
                    }
                    Shape shape = parts[instance->metatype];
                    instance->shape=shape;
                    instance->color = root.get<std::string>("instances." + instance->name + ".color", shape.color);

                }

                
                instance->subtype = root.get<std::string>("instances." + instance->name + ".subtype", "none");
                instance->rotation = root.get<double>("instances." + instance->name + ".rotation", 0.0);
                std::vector<double> position = GetTypes<double>(root, "instances." + instance->name + ".position");
                NC_ASSERT(position.size() > 2);
                instance->x = position[0] * convfactor;
                instance->y = position[1] * convfactor;
                instance->z = position[2] * convfactor;
                std::string state = root.get<std::string>("instances." + instance->name + ".state", "none");
                if(state!="none") 
                    instance->properties["state"]=state;

                //instances[instance].push_back(instance);
                namedinstances[instance->name].push_back(instance);
                typeinstances[instance->type].push_back(instance);
                metatypeinstances[instance->metatype].push_back(instance);
            }

//            for (std::map<std::string, std::vector<Instance> >::iterator it = instances.begin();
//                    it != instances.end(); it++) {
//                std::vector<Instance> instances = (*it).second;
//                for (size_t j = 0; j < instances.size(); j++)
//                    LOG_DEBUG << ((*it).first + DumpInstance(instances[j])).c_str();
//            }
        } catch (std::exception &e) {
            LOG_FATAL << e.what();
        }
    }

    std::vector<boost::shared_ptr<Instance> > Shapes::TypeInstances(std::string type) {
        std::vector<boost::shared_ptr<Instance> > shapes;
        std::map<std::string, std::vector<boost::shared_ptr<Instance> > >::iterator it = typeinstances.find(type);
        if (it == typeinstances.end())
            return shapes;
        shapes = (*it).second;
        return shapes;
    }
    boost::shared_ptr<Instance>  Shapes::NamedInstance(std::string name){
         std::map<std::string, std::vector<boost::shared_ptr<Instance> > >::iterator it = namedinstances.find(name);
        if (it == namedinstances.end())
            throw std::runtime_error("No instance");
        // Should only be one
        if((*it).second.size()==0)
            throw std::runtime_error("No instance");
        return ((*it).second)[0];
    }

    tf::Vector3 Shapes::GetInstancePosition(boost::shared_ptr<Instance> instance) {
        // rotation angle axis + position
        return tf::Vector3(instance->x, instance->y, instance->z);
    }

    tf::Quaternion Shapes::GetInstanceRotation(boost::shared_ptr<Instance> instance) {
        return tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), instance->rotation);
    }
    
    tf::Pose Shapes::GetInstancePose(boost::shared_ptr<Instance> instance) {
        // rotation angle axis + position
        return tf::Pose(GetInstanceRotation(instance),
                tf::Vector3(instance->x, instance->y, instance->z));
    }
    tf::Pose Shapes::GetChildPose(std::string childname) {
        pt::ptree &c = root.get_child(childname);
        std::vector<double> position = GetTypes<double>(c,  "position");
        NC_ASSERT(position.size() > 2);
        double x = position[0] * convfactor;
        double y = position[1] * convfactor;
        double z = position[2] * convfactor;

        return tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                tf::Vector3(x, y, z));
    }
    Shape & Shapes::GetInstanceShape(boost::shared_ptr<Instance> instance) {
        std::map<std::string, Shape >::iterator it = parts.find(instance->metatype);
        assert(it != parts.end());
        return parts[instance->metatype];
    }   
    std::vector<tf::Pose> Shapes::TypeInstanceLocations(std::string sku) {
        std::vector<tf::Pose> poses;
         std::map<std::string, std::vector<boost::shared_ptr<Instance> > >::iterator it = typeinstances.find(sku);
        if (it == typeinstances.end())
            return poses;
        std::vector<boost::shared_ptr<Instance> > v = (*it).second;
        for (size_t i = 0; i< v.size(); i++) {
           boost::shared_ptr<Instance> instance = v[i];
            tf::Pose pose(tf::Quaternion(tf::Vector3(0.0,0.0,1.0), instance->rotation),
                    tf::Vector3(instance->x, instance->y, instance->z));
            poses.push_back(pose);
        }
        return poses;
    }

    std::vector<tf::Pose> Shapes::CopySkuHolderLocations( std::string sku, 
            std::string skuholder) {
        std::vector<tf::Pose> poses;
        std::map<std::string, std::vector<boost::shared_ptr<Instance> > >::iterator it = typeinstances.find(skuholder);
        if (it == typeinstances.end())
            return poses; // fixme this is an error
        std::vector<boost::shared_ptr< ShapeModel::Instance> > v = (*it).second;
 
                
        for (size_t i = 0; i < v.size(); i++) {
            boost::shared_ptr<Instance> instance = v[i];
            if (v[i]->metatype != skuholder)
                return poses;
            tf::Vector3 centroid(instance->x, instance->y, instance->y);
            

            Shape shape = parts[v[0]->metatype];

            BOOST_FOREACH(pt::ptree::value_type &v3,
                    root.get_child("parts." + shape.name + ".contains")) {
                std::string type = root.get<std::string>("parts." + shape.name + ".contains." + v3.first.data() + ".type", "None");
                std::string metatype = root.get<std::string>("parts." + shape.name + ".contains." + v3.first.data() + ".metatype", "None");
                // FIxme this is where db helps - help state- empty, full
                if (type == "holder" && metatype == sku) {
                    std::vector<double> position = GetTypes<double>(root, "parts." + shape.name + ".contains." + v3.first.data() + ".position");
                    tf::Vector3 offset(position[0], position[1], position[2]);
                    offset = centroid + offset;
                    tf::Pose pose = Convert<tf::Vector3, tf::Pose>(offset) * CreatePose(tf::Vector3(0., 0.0, 1.), instance->rotation);
                    poses.push_back(pose);
                }

            }
            return poses;
        }
        // Search each metatype 
        // Given the centroid of the gear vessel. From this compute 4 slots.

    }

};

