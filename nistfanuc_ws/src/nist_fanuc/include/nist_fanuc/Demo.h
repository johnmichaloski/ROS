

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
#include "Controller.h"

class RvizDemo {
protected:
    ros::Subscriber sub;
    ros::NodeHandle &_nh;
    bool bFlag;
    static boost::mutex _flag_mutex;
public:

    RvizDemo(ros::NodeHandle &nh) : _nh(nh) {
        sub = _nh.subscribe("clicked_point", 10, &RvizDemo::callback, this);
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
     IKinematicsSharedPtr _kinematics;
   std::map<tf::Pose, std::vector<double>, cmp_op> mapping;
    typedef std::map<tf::Pose, std::vector<double>, cmp_op>::iterator MapIterator;
	tf::Pose baseOffset;
	Eigen::Affine3d offset00;
public:
    NearestJointsLookup(tf::Pose baseoffset,
            IKinematicsSharedPtr kin): _kinematics(kin), baseOffset(baseoffset) {}
    void Add(tf::Pose pose, std::vector<double> joints) {
        mapping[pose] = joints;
    }

    std::string Dump() {
        std::stringstream str;
        for (MapIterator it = mapping.begin(); it != mapping.end(); it++) {
            str << "Pose Hint " << RCS::DumpPoseSimple((*it).first) <<
                    " = " << RCS::VectorDump<double>((*it).second);

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

    virtual void SetRobotHints() {
    };

    virtual tf::Pose& BaseOffset() {
        return baseOffset;
    }
};

class FanucNearestJointsLookup : public NearestJointsLookup {

public:

    FanucNearestJointsLookup(tf::Pose baseoffset, IKinematicsSharedPtr kin) :
    NearestJointsLookup(baseoffset, kin) {
    }
    virtual void SetRobotHints();
};
class MotomanNearestJointsLookup : public NearestJointsLookup {

public:

    MotomanNearestJointsLookup(tf::Pose baseoffset, IKinematicsSharedPtr kin) :
    NearestJointsLookup(baseoffset, kin) {
    }
    virtual void SetRobotHints();
};
class InlineRobotCommands
{
protected:
	boost::shared_ptr<RCS::CController>_cnc;
    static int crclcommandnum;
    NearestJointsLookup &_hints;
public:
    double mydwell;
    InlineRobotCommands(boost::shared_ptr<RCS::CController> cnc, NearestJointsLookup &hints) : _cnc(cnc), _hints(hints) {
        mydwell = .10; // appears slow
    }

    boost::shared_ptr<RCS::CController> cnc() {
        return _cnc;
    }
    void Pick(RCS::Pose pose, std::string objname);
    void MoveTo(RCS::Pose pose, std::string objname = "");
    void DoDwell(double dwelltime);
    void AddGripperOffset();
    void OpenGripper();
    void CloseGripper();
    void SetGripper(double ee);
    void Place(RCS::Pose pose, std::string objname);
    void MoveObject(std::string objname, RCS::Pose pose, int color);
    void EraseObject(std::string objname);
    void TestRobotCommands();
     void MoveJoints(std::vector<long unsigned int> jointnum,
        std::vector<double> positions);
};


