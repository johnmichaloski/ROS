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


#include "CrclApi.h"
#include "Controller.h"
#include "Globals.h"
#include "Scene.h"
#include "Shape.h"
#include "nist_robotsnc/MotionException.h"

using namespace RCS;
using namespace Conversion;

// Simplistic Testing code
int CrclApi::crclcommandnum = 1;
RCS::Pose CrclApi::retract = RCS::Pose(tf::QIdentity(), tf::Vector3(0, 0, 0.1));

void CrclApi::SetGripper(double ee) {
    // set gripper to 0..1
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = ee;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::CloseGripper() {
    SetGripper(0.75);
}

void CrclApi::OpenGripper() {
    SetGripper(0.45);
}

void CrclApi::DoDwell(double dwelltime) {

    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::MoveTo(RCS::Pose pose, std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;

    //cmd.hint = _hints.FindClosest(pose);
    cmd.finalpose = Convert<tf::Pose, geometry_msgs::Pose> (pose);
    if (!objname.empty()) {
        SceneObject & obj = pScene->Find(objname);
        cmd.partname = objname;
        //cmd.partcolor = (int) pScene->MARKERCOLOR(obj.color);
        cmd.partcolor = obj.rawcolor;

        // lookup gripper close amount from object
    }
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::EraseObject(std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_ERASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::GraspObject(std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_GRASP_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::ReleaseObject(std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_RELEASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::MoveObject(std::string objname, RCS::Pose pose, std::string color) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    cmd.partcolor = Scene::GetColor(color);
    cmd.finalpose = Convert<tf::Pose, geometry_msgs::Pose>(pose);
    _cnc->crclcmds.AddMsgQueue(cmd);
}

void CrclApi::Pick(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();

    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset));
    DoDwell(mydwell);
    MoveTo(RCS::Pose(QBend, offset));
    DoDwell(mydwell);
    CloseGripper();
    DoDwell(mydwell);
    GraspObject(objname);
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
}

void CrclApi::MoveJoints(std::vector<long unsigned int> jointnum,
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

void CrclApi::Place(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();
    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
    DoDwell(mydwell);
    MoveTo(RCS::Pose(QBend, offset), objname);
    ReleaseObject(objname);
    DoDwell(mydwell);
    OpenGripper();
    DoDwell(mydwell);
    MoveTo(retract * RCS::Pose(QBend, offset));

}
