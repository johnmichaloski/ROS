

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
#include "CrclApi.h"
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
    void Exercise(CrclApi *robot);
    VAR(RvizMarker, boost::shared_ptr<CRvizMarker>)
    VAR(Robot, CrclApi*)
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
    void Cycle(boost::shared_ptr<RCS::CController>, CrclApi&);
    OpenHolderSlot FindFreeGearHolder(std::string geartype, bool bFill=true);
    //boost::shared_ptr< ShapeModel::Instance> FindFreeGearHolder(std::string geartype, bool bFill=true);
    //tf::Pose GetFirstSlotHolder(boost::shared_ptr<ShapeModel::Instance> instance, std::string skupart,bool bFill=true);
    boost::shared_ptr< ShapeModel::Instance> FindFreeGear();
    bool IssueRobotCommands(CrclApi & r, bool bSafe=false);
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
    void CheckersGame::PhysicalMove(CrclApi &robot,
            int player,
            int i, int j,
            Checkers::Move m,
            bool doublejump=false);
    void Setup();
    void Play(CrclApi *, CrclApi *);

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


#include "ttt.h"

struct ScriptingDemo {

    ScriptingDemo(ros::NodeHandle & nh) {
        RvizMarker() = boost::shared_ptr<CRvizMarker>(new CRvizMarker(nh));
        RvizMarker()->Init();
        FontColor()=Conversion::ToVector<double>(3,  0.0, 1.0, 0.0);
        Font2WorldScale()=0.1;
    }
    void MarkPose(int flag, tf::Pose pose);
    void Init(CrclApi *robot,
    std::string penholderfile,
    double penholderscale,
    std::string penfile,
    double pencilscale);
    
    void Draw(std::string text,
            unsigned long fontsize = 32,
            double scale = .1,
            tf::Vector3 origin=tf::Vector3(0,0,0),
            tf::Pose transform=tf::Identity());
    VAR(RvizMarker, boost::shared_ptr<CRvizMarker>)
    VAR(Robot, CrclApi*)
    VAR(FontColor, std::vector<double>)
    VAR(Font2WorldScale, double)
    letters script;

};

struct PaintingDemo {

    PaintingDemo(ros::NodeHandle & nh) {
        RvizMarker() = boost::shared_ptr<CRvizMarker>(new CRvizMarker(nh));
        RvizMarker()->Init();
        ImageColor()=Conversion::ToVector<double>(3,  0.0, 1.0, 0.0);
     }
    void Draw(CrclApi *robot,
            std::string imagefilename,
            double scale = .1,
            tf::Pose transform = tf::Identity());

    VAR(RvizMarker, boost::shared_ptr<CRvizMarker>)
    VAR(Robot, CrclApi*)
    VAR(ImageColor, std::vector<double>)

};