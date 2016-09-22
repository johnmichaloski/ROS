

#pragma once

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <ros/ros.h>

#include "RCS.h"
#include "Kinematics.h"
#include "Scene.h"

class BoltDemo {
protected:
    ros::Subscriber sub;
    ros::NodeHandle &_nh;
    bool bFlag;
    static boost::mutex _flag_mutex;
public:

    BoltDemo(ros::NodeHandle &nh) : _nh(nh) {
        sub = _nh.subscribe("clicked_point", 10, &BoltDemo::callback, this);
    }

    bool Ready() {
        return bFlag;
    }

    void Unset() {
        boost::mutex::scoped_lock lock(_flag_mutex);
        bFlag = false;
    }

    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(_flag_mutex);
        geometry_msgs::Point pt = msg->point;
        std::cout << Globals.StrFormat("geometry_msgs::PointStamped=%f:%f:%f\n", pt.x, pt.y, pt.z);
        bFlag = true;
    }
};

class NearestJointsLookup {
protected:

    struct cmp_op {

        bool operator()(const tf::Pose&a, const tf::Pose&b) {
            if(a.getOrigin().x() == b.getOrigin().x())
                return  a.getOrigin().y() < b.getOrigin().y();
            return a.getOrigin().x() < b.getOrigin().x();
        }
    };
    std::map<tf::Pose, std::vector<double>, cmp_op> mapping;
    typedef std::map<tf::Pose, std::vector<double>, cmp_op>::iterator MapIterator;
public:

    void Add(tf::Pose pose, std::vector<double> joints) {
        mapping[pose] = joints;
    }

    std::string Dump() {
        std::stringstream str;
        for (MapIterator it = mapping.begin(); it != mapping.end(); it++) {
            str << "Pose Hint " << RCS::DumpPoseSimple((*it).first) <<
                    " = " << VectorDump<double>((*it).second);

        }
        return str.str();
    }

    std::vector<double> FindClosest(tf::Pose pose) {
        double closest = std::numeric_limits<double>::infinity();

        MapIterator closestit = mapping.end();
        for (MapIterator it = mapping.begin(); it != mapping.end(); it++) {
            double err = ((*it).first.getOrigin() - pose.getOrigin()).length();
            if (err < closest) {
                closestit = it;
                closest = err;
            }
        }
        if (closestit != mapping.end())
            return (*closestit).second;

        return std::vector<double>();
    }
};

extern void TestRobotCommands();
extern void Pick(RCS::Pose pose, std::string objname);
extern void MoveTo(RCS::Pose pose, std::string objname = "");
extern void DoDwell(double dwelltime);
extern void AddGripperOffset();
extern void OpenGripper();
extern void CloseGripper();
extern void SetGripper(double ee);
extern void Place(RCS::Pose pose, std::string objname);
extern void MoveObject(std::string objname, RCS::Pose pose, int color);
extern void EraseObject(std::string objname);
extern void SetRobotHints();
