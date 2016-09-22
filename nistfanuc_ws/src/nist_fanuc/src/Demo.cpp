

#include "Demo.h"
#include "Controller.h"
#include "Globals.h"
#include "Scene.h"
boost::mutex BoltDemo::_flag_mutex;

#ifndef PI_2
#define PI_2 1.5707963268
#endif
double chessdwell = 1.0;
// Simplistic Testing code
static int crclcommandnum = 1;
RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1));
tf::Quaternion QBend(M_PI / 2.0, 0.0, 0.0);
// Checkers hint
//  ToVector<double>(6, 0.27, 0.5, -0.4, 0.0, 0.0, 0.0);
std::vector<double> hint = ToVector<double> (6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);

extern Eigen::Affine3d fanucoffset00;


void SetGripper(double ee) {
    // set gripper to 0..1
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.eepercent = ee;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void CloseGripper() {
    SetGripper(0.75);
}

void OpenGripper() {
    SetGripper(0.45);
}

void AddGripperOffset() {
    // http://robotiq.com/products/adaptive-robot-gripper/
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_SET_GRIPPER_POSE;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(
            RCS::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
            tf::Vector3(0.140, 0.0, -0.017))); // -0.01156)));
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void DoDwell(double dwelltime) {

    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void MoveTo(RCS::Pose pose, std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
#if 0
    if (hint.size() > 0)
        cmd.hint = hint;
    else cmd.hint = ToVector<double>(6, 0.27, 0.7, -0.57, 0.0, 0.0, 0.0);
#endif
    cmd.hint = RCS::Cnc.NearestJoints().FindClosest(pose);
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    if (!objname.empty()) {
        ObjectDB * obj = ObjectDB::Find(objname);
        cmd.partname = objname;
        cmd.partcolor = obj->color;
        // lookup gripper close amount from object
    }
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void EraseObject(std::string objname) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_ERASE_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void MoveObject(std::string objname, RCS::Pose pose, int color) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    cmd.partcolor = color;
    cmd.finalpose = Conversion::RcsPose2GeomMsgPose(pose);
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);
}

void Pick(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();

    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    MoveTo(RCS::Pose(QBend, offset));
    DoDwell(chessdwell);
    CloseGripper();
    DoDwell(chessdwell);
    hint = ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
}

void MoveJoints(std::vector<long unsigned int> jointnum,
        std::vector<double> positions) {
    RCS::CanonCmd cmd;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = RCS::ZeroJointState(6);
    // These tovectors must match double or long depending on template or wont work
    cmd.joints.position = positions; // ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0);
    cmd.jointnum = jointnum; // ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    cmd.bCoordinated = true;
    RCS::Cnc.crclcmds.AddMsgQueue(cmd);

}
// fixme: object has to be richer description

void Place(RCS::Pose pose, std::string objname) {

    tf::Vector3 offset = pose.getOrigin();
    // Retract
    MoveTo(retract * RCS::Pose(QBend, offset), objname);
    DoDwell(chessdwell);
    MoveTo(RCS::Pose(QBend, offset), objname);
    DoDwell(chessdwell);
    OpenGripper();
    DoDwell(chessdwell);
    MoveTo(retract * RCS::Pose(QBend, offset));

}

// Major problem is the KDL expects a reasonable hint at the joint solution for IK
// And with large jumps in bang-bang control, not realistic yet.
/// Either ik or more trajectory control with spacing between joint updates small so KDL happy

void SetRobotHints() {
    double xmin, xmax, ymin, ymax;
    tf::Vector3 vec;
    //tf::Vector3 xvmax(0.7, 0.0  ,0.0);
    tf::Pose goalpose;
    tf::Pose baseOffset = Conversion::Affine3d2RcsPose(fanucoffset00);
    std::vector<double> joints;

    hint = ToVector<double>(6, -0.160, 1.8, 1.22, 0.0, 0.0, 0.0);
    vec = tf::Vector3(0.0, .45, 0.0);
    do {
        vec += tf::Vector3(0.0, .1, 0.0);
        goalpose = tf::Pose(QBend, vec)*baseOffset;
        joints = Cnc.Kinematics()->IK(goalpose, hint);
        if (joints.size() > 0) {
            RCS::Cnc.NearestJoints().Add(goalpose, joints);
            hint = joints;
        }
    } while (joints.size() > 0);
    ymax = vec.y() - 0.01;

    vec = tf::Vector3(0.0, .45, 0.0);
    hint = ToVector<double>(6, 0.160, 1.8, 1.22, 0.0, 0.0, 0.0);
    do {
        vec += tf::Vector3(0.0, -.1, 0.0);
        goalpose = tf::Pose(QBend, vec)*baseOffset;
        joints = Cnc.Kinematics()->IK(goalpose, hint);
        if (joints.size() > 0) {
            RCS::Cnc.NearestJoints().Add(goalpose, joints);
            hint = joints;
        }
    } while (joints.size() > 0 && vec.getY() > 0.0);
    ymin = vec.y() + 0.01;

    hint = ToVector<double>(6, 0.0, 1.33, 0, 0.0, 0.0, 0.0);
    vec = tf::Vector3(0.45, 0.0, 0.0);
    do {
        vec += tf::Vector3(.1, 0.0, 0.0);
        goalpose = tf::Pose(QBend, vec)*baseOffset;
        joints = Cnc.Kinematics()->IK(goalpose, hint);
        if (joints.size() > 0) {
            RCS::Cnc.NearestJoints().Add(goalpose, joints);
            hint = joints;
        }
    } while (joints.size() > 0);
    xmax = vec.x() - 0.01;

    hint = ToVector<double>(6, 0.0, 1.33, 0, 0.0, 0.0, 0.0);
    vec = tf::Vector3(0.45, 0.0, 0.0);
    do {
        vec += tf::Vector3(-.1, 0.0, 0.0);
        goalpose = tf::Pose(QBend, vec)*baseOffset;
        joints = Cnc.Kinematics()->IK(goalpose, hint);
        if (joints.size() > 0) {
            RCS::Cnc.NearestJoints().Add(goalpose, joints);
            hint = joints;
        }
    } while (joints.size() > 0 && vec.getX() > 0.0);
    xmin = vec.x() + 0.01;
    
    // Bolt locations

    RCS::Cnc.NearestJoints().Add(RCS::Pose(QBend, tf::Vector3(0.25, -0.45, 0.04)) * baseOffset,
            ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0));
    RCS::Cnc.NearestJoints().Add(RCS::Pose(QBend, tf::Vector3(0.25, 0.45, 0.04)) * baseOffset,
            ToVector<double> (6, 0.99, 0.78, 0.0, 0.0, 0.0, 0.0));

    RCS::Cnc.NearestJoints().Add(RCS::Pose(QBend,
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245)) * baseOffset,
            ToVector<double> (6, -0.27, 0.8, -0.55, 0.0, 0.0, 0.0));
    
    // Bolt holder locations
    RCS::Cnc.NearestJoints().Add(RCS::Pose(QBend,   
            tf::Vector3(0.390496134758, -0.101964049041, 0.0245)) * 
           baseOffset,
          ToVector<double> (6, -0.27, 0.95, -0.14, 0.0, 0.0, 0.0));  
//        tf::Vector3(0.38512814045, 0.00476559251547, 0.0245),
//        tf::Vector3(0.497517108917, -0.106719098985, 0.0245),
//        tf::Vector3(0.497517108917, 0.00590129941702, 0.0245)

                   
    LOG_DEBUG << RCS::Cnc.NearestJoints().Dump();
}

void TestRobotCommands() {
    // Finish queuing commands before handling them....
    boost::mutex::scoped_lock lock(cncmutex);
    static double dwelltime = 3.0;
    std::vector<long unsigned int> vjointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2));

    RCS::CanonCmd cmd;

    // Warm up
    MoveJoints(vjointnum, ToVector<double>(6, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    DoDwell(dwelltime);
    MoveJoints(vjointnum, ToVector<double>(6, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0));
    DoDwell(dwelltime);

    
    RCS::Pose pickpose;
    // Bolt slots don't move for now
    tf::Vector3 boltslots[4] = {
        tf::Vector3(0.390496134758, -0.101964049041, 0.0245),
        tf::Vector3(0.38512814045, 0.00476559251547, 0.0245),
        tf::Vector3(0.497517108917, -0.106719098985, 0.0245),
        tf::Vector3(0.497517108917, 0.00590129941702, 0.0245)
    };
    std::vector<int> bolts;
    for (size_t i = 0; i < 4; ++i) bolts.push_back(i); // 0 1 2 3 
    std::random_shuffle(bolts.begin(), bolts.end());

    for (size_t i = 0; i < 4; i++) {
        std::string boltname = Globals.StrFormat("bolt%d", i+1);
        int bolt = bolts.back();  bolts.pop_back();
        ObjectDB * obj = ObjectDB::Find(boltname);
        Eigen::Affine3d affpose = Conversion::convertPointToPose(obj->pose.translation() + Eigen::Vector3d(0.0, 0.0, 0.015002));

        pickpose = RCS::Pose(QBend, Conversion::vectorEigenToTF(affpose.translation())); // tf::Vector3(0.25, -0.45, 0.04 + 0.015002));

        tf::Vector3 offset = pickpose.getOrigin();

        // Retract
        //hint = ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
        MoveTo(retract * RCS::Pose(QBend, offset));
        DoDwell(chessdwell);
        MoveTo(RCS::Pose(QBend, offset));
        DoDwell(chessdwell);
        CloseGripper();
        DoDwell(chessdwell);
        //hint = ToVector<double> (6, -0.99, 0.78, 0.0, 0.0, 0.0, 0.0);
        MoveTo(retract * RCS::Pose(QBend, offset), boltname);

        //hint = ToVector<double> (6, -0.27, 0.8, -0.55, 0.0, 0.0, 0.0);
         RCS::Pose placepose = RCS::Pose(QBend,boltslots[bolt])*
                 Conversion::Affine3d2RcsPose(fanucoffset00);
         Place(placepose, boltname);
    }
}
