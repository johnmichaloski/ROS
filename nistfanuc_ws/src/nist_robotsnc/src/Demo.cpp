

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
#include "Shape.h"
#include "nist_robotsnc/MotionException.h"

using namespace RCS;

#ifndef PI_2
#define PI_2 1.5707963268
#endif

boost::mutex RvizDemo::_flag_mutex;



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
        ObjectDB * obj = pScene->Find(objname);
        cmd.partname = objname;
        cmd.partcolor = (int) pScene->MARKERCOLOR(obj->color);
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

void InlineRobotCommands::MoveObject(std::string objname, RCS::Pose pose, std::string color) {
    RCS::CanonCmd cmd;
    cmd.crclcommand = CanonCmdType::CANON_DRAW_OBJECT;
    cmd.crclcommandnum = crclcommandnum++;
    cmd.partname = objname;
    cmd.partcolor = pScene->MARKERCOLOR(color);
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
/////////////////////////////////////////////////////////////////////////

GearDemo::GearDemo(ros::NodeHandle & nh, std::string pkgpath, tf::Pose offset) :
_nh(nh), _path(pkgpath), _baseoffset(offset), rvizdemo(nh) {

    _jsonpath = pkgpath + "/config/shapes.json";
    _shapes.ParseJsonFile(_jsonpath);

}

bool GearDemo::IssueRobotCommands(InlineRobotCommands & r) {
    // Finish queuing commands before handling them....
    boost::mutex::scoped_lock lock(cncmutex);
    static double dwelltime = 1.0;
    std::vector<long unsigned int> vjointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    //RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2));

    RCS::CanonCmd cmd;
    RCS::Pose pickpose;

    boost::shared_ptr< ShapeModel::Instance> instance;
    if ((instance = FindFreeGear()) == NULL)
        return false;

    std::string gearname = instance->name;
    
    // Ok we will move this gear - mark as no longer free standing
    instance->properties["state"] = "stored";
    
    ObjectDB * obj = pScene->Find(instance->name);
    NC_ASSERT(obj != NULL);
    
    Eigen::Affine3d affpose = Conversion::convertPointToPose(obj->pose.translation() + obj->gripperoffset.translation());

    pickpose = RCS::Pose(r.QBend, Conversion::vectorEigenToTF(affpose.translation()));

    tf::Vector3 offset = pickpose.getOrigin();

    // Retract
    r.MoveTo(retract * RCS::Pose(r.QBend, offset));
    r.DoDwell(r.mydwell);
    r.MoveTo(RCS::Pose(r.QBend, offset));
    r.DoDwell(r.mydwell);
    r.CloseGripper();
    r.DoDwell(r.mydwell);
    r.MoveTo(retract * RCS::Pose(r.QBend, offset), gearname);

    tf::Pose slotpose;

    try {
        GearDemo::OpenHolderSlot holderslot = FindFreeGearHolder(instance->metatype);
        slotpose = holderslot.second;
        boost::shared_ptr< ShapeModel::Instance> vinstance = holderslot.first;
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(vinstance);
        slotpose = gearpose * slotpose;
    } catch (...) {
        return false;
    }

    RCS::Pose placepose = RCS::Pose(r.QBend, slotpose.getOrigin()); // fixme: what if gear rotated
    r.Place(placepose, gearname);
    r.DoDwell(r.mydwell);

    std::vector<unsigned long> jointnum(r.cnc()->Kinematics()->NumJoints());
    std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
    r.MoveJoints(jointnum, r.cnc()->NamedJointMove["Safe"]);
    return true;
}

boost::shared_ptr< ShapeModel::Instance> GearDemo::FindFreeGear() {
    std::string type = "gear";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

        if (instances[i]->properties["state"] == "free")
            return instances[i];
    }
    return NULL;
}

GearDemo::OpenHolderSlot GearDemo::FindFreeGearHolder(std::string geartype, bool bFill) {

    std::string vesseltype = "holder";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > vinstances = _shapes.TypeInstances(vesseltype);
    for (size_t i = 0; i < vinstances.size(); i++) {
        std::vector<std::string> slots = _shapes.GetChildrenNames("parts." + vinstances[i]->metatype + ".contains");
        std::vector<std::string> slotnames = _shapes.GetShapeChildrenNames(vinstances[i]->metatype, "contains");
        for (size_t j = 0; j < slotnames.size(); j++) {
            // Check that type == holder and metatype == metatype
            std::string type = _shapes.GetChildValue<std::string>(slotnames[j] + ".type");
            std::string metatype = _shapes.GetChildValue<std::string>(slotnames[j] + ".metatype");
            if (type != "holder" || metatype != geartype)
                continue;
            std::string statechild = vinstances[i]->propname + ".contains." + slots[j] + ".state";
            std::string state = _shapes.GetChildValue<std::string>(statechild);
            if (state == "full")
                continue;
            if (bFill) {
                _shapes.PutChildValue<std::string>(statechild, "full");
            }
            return std::make_pair(vinstances[i], _shapes.GetChildPose(slotnames[j]));
        }
    }
    throw MotionException(1030, "");
}

void GearDemo::Setup() {

    // Find and make scene objects of all free gears
    std::string type = "gear";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

//        if (instances[i]->properties["state"] != "free")
//            continue;
        
        std::string gearname = instances[i]->name; // Globals.StrFormat("%s%d", sku.c_str(), i + 1);
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(instances[i]);
 
        ShapeModel::Shape shape = _shapes.GetInstanceShape(instances[i]);
        ObjectDB *obj = pScene->CreateMesh(gearname, "gear",
                pScene->gid++,
                Conversion::tfPose2Affine3d(gearpose),
                shape.meshfile,
                shape.color,
                shape.scale);
        std::vector<double> graspoffset = _shapes.GetChildValues<double>("parts." + shape.name + ".gripper.offset");
        obj->gripperoffset = Conversion::tfPose2Affine3d(Conversion::CreateRPYPose(graspoffset));
        obj->instance = instances[i];
        //        pScene->CreateMarker("maker",
        //                Conversion::tfPose2Affine3d(gearpose * Conversion::CreatePose(tf::Vector3(0.0, 0.0, 0.04))
        //                ),
        //                "GREEN");
    }

    // Find and make scene objects of all gear holders (kits or vessel)  
    type = "holder";
    instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

        std::string holdername = instances[i]->name;

        // adjust pose into robot coordinates (baseoffset) 
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(instances[i]);

        ShapeModel::Shape shape = _shapes.GetInstanceShape(instances[i]);
        //tf::Vector3 pos = shapes.GetInstancePosition(instances[i]); // +tf::Vector3(0.0455, 0.0356,0.0);
        //        pos.rotate(tf::Vector3(0.0,0.0,1.0), Deg2Rad(-90.0));
        ObjectDB *obj = pScene->CreateMesh(holdername,
                "gearholder",
                pScene->gid++,
                Conversion::tfPose2Affine3d(gearpose),
                shape.meshfile,
                shape.color,
                shape.scale);
        obj->instance = instances[i];
#if 0
        try {
            //            ShapeModel::Instance outline = _shapes.NamedInstance("outline_" + holdername);
            //
            //            double width = _shapes.GetInstanceValue<double>(outline, "width");
            //            double height = _shapes.GetInstanceValue<double>(outline, "height");
            //            pScene->CreateWireframeCuboid("gearholderoutline",
            //                    "trayoutline",
            //                    Conversion::tfPose2Affine3d(gearpose),
            //                    width, height, .04, "GREEN");

            // 
            // Now  fill in slots where gears are to go  in array sku_gear_vessel
            std::vector<std::string> slotnames = _shapes.GetShapeChildrenNames(instances[i].metatype, "contains");
            // Do they match - skip for now
            for (size_t j = 0; j < s // FIXME: check that this is a mesh
                    lotnames.size(); j++) {
                tf::Pose slotpose = _shapes.GetChildPose(slotnames[j]);
                LOG_DEBUG << "before slotpose " << RCS::DumpPoseSimple(gearpose).c_str();
                // gear pose will already include rotation
                slotpose = gearpose * slotpose;
                LOG_DEBUG << "after slotpose " << RCS::DumpPoseSimple(slotpose).c_str();
                pScene->CreateMarker("marker",
                        Conversion::tfPose2Affine3d(slotpose),
                        "GREEN");
                pScene->sku_gear_vessel.push_back(slotpose);
            }


        } catch (...) {
        }
#endif
    }
    // Debug: LOG_DEBUG << ObjectDB::DumpDB();   
}

void GearDemo::Reset() {
//    pScene->ClearScene(); // erases all markers
//    Setup(); // recreates all markers
//    pScene->DrawScene(); // draws rviz scene with markers
}

void GearDemo::Cycle(boost::shared_ptr<RCS::CController> nc, InlineRobotCommands&robot) {
    ros::Rate r(50);
//    nc->Suspend();
    while (IssueRobotCommands(robot) != NULL) {
 //       nc->Start();
        while (nc->IsBusy()) {
            if (rvizdemo.Clicked()) {
                RCS::Thread::SuspendAll();
                while (!rvizdemo.Clicked())
                    r.sleep();
                RCS::Thread::ResumeAll();
            }
            ros::spinOnce();
            ros::spinOnce();
            r.sleep();
        }
//        nc->Suspend();
    }
}
//////////////////////////////////////////////////////////////////////////
#include "Checkerboard.h"

CheckersGame::CheckersGame(ros::NodeHandle & nh) : _nh(nh) {
    rvizgame = boost::shared_ptr<RvizCheckers> (new RvizCheckers(nh));

}

void CheckersGame::Setup() {
    //    RvizDemo rvizdemo(_nh);
    pScene->InitScene();
    rvizgame->RvizSetup();
    pScene->DrawScene(); // Debug: LOG_DEBUG << ObjectDB::DumpDB();
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
