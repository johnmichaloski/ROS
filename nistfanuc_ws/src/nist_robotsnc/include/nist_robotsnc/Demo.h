

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
#include "Conversions.h"

/**
 * \brief RvizDemo provides some synchronization primitives via rviz.
 * Uses PublishPoint to listen and set flags when pushed. 
 */
class RvizDemo {
protected:
    ros::Subscriber sub; /**<  clicked_point subscriber */
    ros::NodeHandle &_nh; /**<  reference to ROS node handle */
    bool bFlag; /**<  flag indicating clicked_point has occurred. Must be manually reset. */
    bool bClicked; /**<  flag indicating clicked_point has occurred. Auto reset. */
    static boost::mutex _flag_mutex; /**<  mutex for one callback running at time */
public:

    /*!
     * \brief Constructor of rviz synchronization primitives requires node handle.
     * susbscribes to rviz clicked_point topic
     * \param nh is ROS node handle.
     */
    RvizDemo(ros::NodeHandle &nh) : _nh(nh), bClicked(false), bFlag(false) {
        sub = _nh.subscribe("clicked_point", 10, &RvizDemo::callback, this);
    }

    /*!
     * \brief Ready returns a boolean indicating whether Rviz clicked_point has been clicked.
     * \return  T       returns clicked_point ready flag.
     */
    bool Ready() {
        return bFlag;
    }

    /*!
     * \brief Unsets  a boolean indicating whether Rviz clicked_point has been clicked.
     */
    void Unset() {
        boost::mutex::scoped_lock lock(_flag_mutex);
        bFlag = false;
    }

/*!
     * \brief Ready returns a boolean indicating whether Rviz clicked_point has been clicked. Resets flag to false.
     * \return  T       returns clicked_point detected flag.
     */
    bool Clicked() {
        bool b = bClicked;
        bClicked = false;
        return b;
    }


    /*!
     * \brief  Rviz clicked_point subscribed callback has been clicked. 
     * Sets bFlag and  bClickedflag to true.
     * \param  msg  clicked_point topic callback message.
     */
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(_flag_mutex);
        geometry_msgs::Point pt = msg->point;
        std::cout << Globals.StrFormat("geometry_msgs::PointStamped=%f:%f:%f\n", pt.x, pt.y, pt.z);
        bFlag = true;
        bClicked = true;
    }
};

/**
 * \brief InlineRobotCommands provides some inline Crcl commands that are queued to given CNC.
 */
class InlineRobotCommands {
protected:
    boost::shared_ptr<RCS::CController>_cnc;
    static int crclcommandnum; /**<  crcl command number for this robot */
public:
    double mydwell; /**<  global dwell time */
    tf::Quaternion QBend; /**< rotation so end effect is facing down (as opposed to sideways)*/

    /*!
     * \brief Constructor of commands requires reference to Controller object.
     * \param cnc is pointer to CController instance.
     */
    InlineRobotCommands(boost::shared_ptr<RCS::CController> cnc) : _cnc(cnc) {
        mydwell = .50;
        QBend = cnc->QBend();
    }

    /*!
     * \brief Return pointer to CNC instance of this robot command object.
     * \return cnc is pointer to CController instance.
     */
    boost::shared_ptr<RCS::CController> cnc() {
        return _cnc;
    }
    /*!
     * \brief Robot picks up an object at pose with given objname.
     * \param pose of the given object to pick up.
     * \param objname name of the object that is being picked up.
     */
    void Pick(RCS::Pose pose, std::string objname);
    /*!
     * \brief Robot moves to given Cartesian pose, and may move object .
     * \param pose of the given object to pick up.
     * \param objname name of the object that is being picked up.
     */
    void MoveTo(RCS::Pose pose, std::string objname = "");
    /*!
     * \brief Robot dwells for given dwell time in seconds. .
     * \param dwelltime time to dwell in seconds.
     * */
    void DoDwell(double dwelltime);
    //void AddGripperOffset();
    /*!
     * \brief Robot opens gripper. .
     */
    void OpenGripper();
    /*!
     * \brief Robot closes gripper. .
     */
    void CloseGripper();
    /*!
     * \brief Set the robot gripper to the given percentage (from 0..1). .
     * \param ee end effector percentage close(0)..open(1). Note gripper could be closed at 1!
     */
    void SetGripper(double ee);
    /*!
     * \brief Robot places up an object at pose with given objname. 
     * Retracts to given retraction offset from place pose.
     * \param pose of the given object to pick up.
     * \param objname name of the object that is being picked up.
     */
    void Place(RCS::Pose pose, std::string objname);

    /*!
     * \brief Move object with given objname to given pose.
     * \param objname name of the object that is being picked up.
     * \param pose of the given object to pick up.
     * \param color sets the object color
     * 
     */
    void MoveObject(std::string objname, RCS::Pose pose, std::string color);
    void GraspObject(std::string objname);
    void ReleaseObject(std::string objname);
    /*!
     * \brief Erases object with given objname .
     * \param objname name of the object that is being picked up.
     */
    void EraseObject(std::string objname);
    /*!
     * \brief Robot moves joints as defined joint number vector to positions.
     * Coordinated joint motion "assumed".
     * \param jointnum is a list of joints to move.
     * \param positions is final joint position 
     */
    void MoveJoints(std::vector<long unsigned int> jointnum,
            std::vector<double> positions);


};
#include "RvizMarker.h"

struct ExerciseDemo {

    ExerciseDemo(ros::NodeHandle & nh) {
        RvizMarker() = boost::shared_ptr<CRvizMarker>(new CRvizMarker(nh));
        RvizMarker()->Init();
        GoodColor()=Conversion::ToVector<double>(3,  0.0, 1.0, 0.0);
        BadColor()=Conversion::ToVector<double>(3, 1.0, 0.0, 0.0);
        TrapColor()=Conversion::ToVector<double>(3, 1.0, 0.4, 1.0);

    }
    void MarkPose(int flag, tf::Pose pose);
    void Exercise(InlineRobotCommands *robot);
    VAR(RvizMarker, boost::shared_ptr<CRvizMarker>)
    VAR(Robot, InlineRobotCommands*)
    VAR(GoodColor, std::vector<double>)
    VAR(BadColor,  std::vector<double>)
    VAR(TrapColor, std::vector<double>)

    
};

// Gear Demo 
#include "Shape.h"
struct GearDemo {
    typedef std::pair<boost::shared_ptr< ShapeModel::Instance>, tf::Pose> OpenHolderSlot;
    GearDemo(ros::NodeHandle & nh, std::string pkgpath, tf::Pose offset);
    void Setup();
    void Reset();
    void Cycle(boost::shared_ptr<RCS::CController>, InlineRobotCommands&);
    OpenHolderSlot FindFreeGearHolder(std::string geartype, bool bFill=true);
    //boost::shared_ptr< ShapeModel::Instance> FindFreeGearHolder(std::string geartype, bool bFill=true);
    //tf::Pose GetFirstSlotHolder(boost::shared_ptr<ShapeModel::Instance> instance, std::string skupart,bool bFill=true);
    boost::shared_ptr< ShapeModel::Instance> FindFreeGear();
    bool IssueRobotCommands(InlineRobotCommands & r, bool bSafe=false);
protected:
    ShapeModel::Shapes _shapes;
    ros::NodeHandle & _nh;
    std::string _path;
    tf::Pose _baseoffset;
    RvizDemo rvizdemo;
    std::string _jsonpath;
};


// Checkers game - demo defined in Demo.cpp
#include "RvizCheckers.h"
class CheckersGame {
    boost::shared_ptr<RvizCheckers> rvizgame;
    ros::NodeHandle & _nh;
public:
    CheckersGame(ros::NodeHandle & nh);
    void CheckersGame::PhysicalMove(InlineRobotCommands &robot,
            int player,
            int i, int j,
            Checkers::Move m,
            bool doublejump=false);
    void Setup();
    void Play(InlineRobotCommands *, InlineRobotCommands *);

    boost::shared_ptr<RvizCheckers> RvizGame() {
        return rvizgame;
    }
    double z;
    double table_length;
    double table_width;
    double table_height;
    std::vector<double> xyz;
    boost::shared_ptr<CRvizMarker> rvizMarker;
    
};