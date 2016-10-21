

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


#include "Demo.h"
#include "Controller.h"
#include "Globals.h"
#include "Scene.h"
using namespace RCS;

#ifndef PI_2
#define PI_2 1.5707963268
#endif

boost::mutex RvizDemo::_flag_mutex;

static tf::Vector3 boltslots[4] = {
    tf::Vector3(-0.113, -0.107, 0.0245),
    tf::Vector3(0.00, -0.107, 0.0245),
    tf::Vector3(-0.113, 0.007, 0.0245),
    tf::Vector3(0.0, 0.007, 0.0245)
};

// Motoman - assuming at 0, .5, 0
// 0 0 0 = 1.15 -1.74 0 1.34 .22 0 0  (hint)

// Simplistic Testing code
int InlineRobotCommands::crclcommandnum = 1;
RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1));
// Checkers hint
//  ToVector<double>(6, 0.27, 0.5, -0.4, 0.0, 0.0, 0.0);
//std::vector<double> hint = ToVector<double> (6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);

void InlineRobotCommands::SetGripper(double ee) {
    // set gripper to 0..1
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = ee;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void InlineRobotCommands::CloseGripper() {
    SetGripper(0.75);
}

void InlineRobotCommands::OpenGripper() {
    SetGripper(0.45);
}

void InlineRobotCommands::DoDwell(double dwelltime) {

    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void InlineRobotCommands::MoveTo(RCS::Pose pose, std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;

    //cmd.hint = _hints.FindClosest(pose);
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    if (!objname.empty()) {
        ObjectDB * obj = ObjectDB::Find(objname);
        cmd.partname = objname;
        cmd.partcolor = obj->color;
        // lookup gripper close amount from object
    }
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void InlineRobotCommands::EraseObject(std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_ERASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void InlineRobotCommands::MoveObject(std::string objname, RCS::Pose pose, int color) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    cmd.partcolor = color;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void InlineRobotCommands::Pick(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();

    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset));
    DoDwell(mydwell);
    MoveTo(RCS::Pose(QBend, offset));
    DoDwell(mydwell);
    CloseGripper();
    DoDwell(mydwell);
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
}

void InlineRobotCommands::MoveJoints(std::vector<long unsigned int> jointnum,
        std::vector<double> positions) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = RCS::ZeroJointState(jointnum.size());
    cmd.joints.position = positions;
    cmd.jointnum = jointnum; // ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    _cnc->crclcmds.AddMsgQueue(cmd);

}
// fixme: object has to be richer description

void InlineRobotCommands::Place(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();
    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
    DoDwell(mydwell);
    MoveTo(RCS::Pose(QBend, offset), objname);
    DoDwell(mydwell);
    OpenGripper();
    DoDwell(mydwell);
    MoveTo(retract * RCS::Pose(QBend, offset));

}

void InlineRobotCommands::TestRobotCommands() {
    // Finish queuing commands before handling them....
    boost::mutex::scoped_lock lock(cncmutex);
    static double dwelltime = 1.0;
    std::vector<long unsigned int> vjointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    //RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2));

    RCS::CanonCmd cmd;

    // Warm up
    //    MoveJoints(vjointnum, ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    //    DoDwell(dwelltime);
    //    MoveJoints(vjointnum, ToVector<double>(6, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    //    DoDwell(dwelltime);


    RCS::Pose pickpose;
    // Bolt slots don't move for now - used ClickPoint on Rviz to find

    std::vector<int> bolts;
    for (size_t i = 0; i < 4; ++i) bolts.push_back(i); // 0 1 2 3 
    std::random_shuffle(bolts.begin(), bolts.end());

    for (size_t i = 0; i < 4; i++) {
        std::string boltname = Globals.StrFormat("bolt%d", i + 1);
        int bolt = bolts.back();
        bolts.pop_back();
        ObjectDB * obj = ObjectDB::Find(boltname);
        Eigen::Affine3d affpose = Conversion::convertPointToPose(obj->pose.translation() + Eigen::Vector3d(0.0, 0.0, 0.015002));

        pickpose = RCS::Pose(QBend, Conversion::vectorEigenToTF(affpose.translation())); // tf::Vector3(0.25, -0.45, 0.04 + 0.015002));

        tf::Vector3 offset = pickpose.getOrigin();

        // Retract
        MoveTo(retract * RCS::Pose(QBend, offset));
        DoDwell(mydwell);
        MoveTo(RCS::Pose(QBend, offset));
        DoDwell(mydwell);
        CloseGripper();
        DoDwell(mydwell);
        MoveTo(retract * RCS::Pose(QBend, offset), boltname);

        RCS::Pose placepose = RCS::Pose(QBend, boltslots[bolt]); // * this->_cnc->basePose();
        Place(placepose, boltname);
        DoDwell(mydwell);

        std::vector<unsigned long> jointnum(cnc()->Kinematics()->NumJoints());
        std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
        MoveJoints(jointnum, cnc()->NamedJointMove["Safe"]);
    }
}

void GearDemoSetup(ros::NodeHandle & nh) {
    RvizDemo rvizdemo(nh);

    InitScene();
    DrawScene();

    // Debug: LOG_DEBUG << ObjectDB::DumpDB();
#if 1
    // This waits for a publish point from rviz before proceeding
    while (!rvizdemo.Ready()) {
        ros::Duration(1.0).sleep();
    }
    ros::Duration(5.0).sleep();
#endif   
}

void GearDemoReset() {
    ClearScene();
    NewScene();
    DrawScene();
}

void GearDemoCycle(boost::shared_ptr<RCS::CController> nc, InlineRobotCommands&robot) {
    ros::Rate r(50);
    if (nc->crclcmds.SizeMsgQueue() == 0) {
        robot.TestRobotCommands();
    }
    while (nc->IsBusy()) {
        ros::spinOnce();
        ros::spinOnce();
        r.sleep();
    }
}

#include "Checkerboard.h"

CheckersGame::CheckersGame(ros::NodeHandle & nh): _nh(nh)
{
     rvizgame = boost::shared_ptr<RvizCheckers> (new RvizCheckers(nh));
   
}
void CheckersGame::Setup() {
//    RvizDemo rvizdemo(_nh);
    InitScene();
    rvizgame->RvizSetup();
    DrawScene(); // Debug: LOG_DEBUG << ObjectDB::DumpDB();
}

void CheckersGame::Play(InlineRobotCommands * red, InlineRobotCommands * black) {
    //InlineRobotCommands * Ncs[2]={&nccmds[0], &nccmds[1]};
    //InlineRobotCommands * Ncs[2]={&fanucrobot, &fanucrobot};
    //InlineRobotCommands * Ncs[2]={&motomanrobot, &motomanrobot};

    // Play checkers - only move markers, no robot interaction
    Checkers::Move from, to;
    int player;
    for (size_t i = 0; i < 40; i++) {
        if (rvizgame->CheckersMove(player, from, to))
            break;
        rvizgame->Game().printDisplayFancy(rvizgame->Game().Board());
        if (player == Checkers::RED) {
            LOG_DEBUG << "RED Move " << red->cnc()->Name().c_str();
            rvizgame->PhysicalMove(*red, player, from.row, from.col, to);
        } else {
            LOG_DEBUG << "BLACK Move " << black->cnc()->Name().c_str();
            rvizgame->PhysicalMove(*black, player, from.row, from.col, to);
        }

#if 0
        // Synchronize with rviz let PublishPoint pause execution
        if (rvizdemo.Clicked()) {
            for (size_t j = 0; j < Ncs.size(); j++)
                Ncs[0]->cnc()->Suspend();
            while (1) {
                ros::spinOnce();
                ros::Duration(0.2).sleep();
                if (rvizdemo.Clicked())
                    break;
            }
            for (size_t j = 0; j < Ncs.size(); j++)
                Ncs[0]->cnc()->Resume();

        }
#endif
        ros::spinOnce();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}