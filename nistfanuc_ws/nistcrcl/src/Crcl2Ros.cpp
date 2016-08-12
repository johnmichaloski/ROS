


#include "Crcl2Ros.h"
#include <urdf/model.h>
//#include <boost/regex.hpp>
using namespace urdf;
/**

 * 
 > catkin build -DCMAKE_BUILD_TYPE=Deb
 > roscore

 * 
 *    
#include <urdf/model.h>
using namespace urdf;
 
 * // Parse robot description
     const std::string model_param_name = "robot_description";
    bool res = _nh.hasParam(model_param_name);
    std::string robot_model_str = "";
    if (!res || !_nh.getParam(model_param_name, robot_model_str)) {
        ROS_ERROR_NAMED("nistcrcl", "Robot descripion couldn't be retrieved from param server.");
        return ;
    }
    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse robot_description urdf");

    }
    boost::shared_ptr<const Link> root_link = model.getRoot();
    traverse_tree(root_link, jointnames);
    // Get Joint Names
    //_nh.getParam("controller_joint_names", crclinterface->crclwm.jointnames);
    std::cout << VectorDump<std::string> ( crclinterface->crclwm.jointnames);
 
 
 * 
 * 
 * #include <boost/regex.h>
    jointnames.clear();
    std::string pattern = "<joint name=\"(.+?)\">";
    boost::regex joint_regex(pattern);
    boost::sregex_iterator end;
    boost::sregex_iterator it(robot_model_str.begin(), robot_model_str.end(), joint_regex);
    for (; it != end; ++it) {
        jointnames.push_back( it->str() );
    }
 */

namespace RCS {
    CMessageQueue<RCS::CanonCmd> cmds;
    CanonWorldModel wm; // for motion control planning wm
}
boost::mutex CCrcl2RosMsg::cncmutex;

size_t num_joints;
size_t num_links;

bool traverse_tree(boost::shared_ptr<const Link> link, std::vector<std::string>&joint_names, int level = 0) {
    ROS_INFO("Traversing tree at level %d, link size %lu", level, link->child_links.size());
    level += 2;
    bool retval = true;
    for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
        ++num_links;
        if (*child && (*child)->parent_joint) {
            ++num_joints;

            //http://docs.ros.org/diamondback/api/urdf/html/classurdf_1_1Joint.html#a74b855c338e9e5cdc2e0760e6aca738b
            joint_names.push_back((*child)->parent_joint->name);

            // FIXME: should not include fixed, and only group by manipulator
            // recurse down the tree
            retval &= traverse_tree(*child, joint_names, level);
        } else {
            ROS_ERROR("root link: %s has a null child!", link->name.c_str());
            return false;
        }
    }
    // no more children
    return retval;
};

void CCrcl2RosMsg::StatusCallback(const nistcrcl::CrclStatusMsg::ConstPtr& statusmsg) {
    ROS_INFO("CCrcl2RosMsg::StatusCallbac");
    //crclinterface->crclwm.CommandID() // THis is assigned a # to fit with CRCL XML communication;
    crclinterface->crclwm.StatusID() = (unsigned long long) statusmsg->crclcommandnum;
    //    crclinterface->crclwm.CommandStatus() = (Crcl::CommandStateEnum) statusmsg->crclcommandstatus;


    crclinterface->crclwm.Update((RCS::Pose&) statusmsg->statuspose);
    crclinterface->crclwm.Update((JointState&) statusmsg->statusjoints);
    crclinterface->crclwm.Gripper().Position() = statusmsg->eepercent;
}

void CCrcl2RosMsg::Init() {

    // Controller instantiatio of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<Crcl::CrclDelegateInterface>(
            new Crcl::CrclDelegateInterface());
    crclinterface->SetAngleUnits("DEGREE");
    crcl_pub = _nh.advertise<nistcrcl::CrclCommandMsg>("crcl_command", 10);
    crcl_sub = _nh.subscribe("crcl_status", 1000, &CCrcl2RosMsg::StatusCallback, this);

    // Couple code attempts at reading from robot_description parameter - see above
    crclinterface->crclwm.jointnames.clear();

    _nh.getParam("controller_joint_names", crclinterface->crclwm.jointnames);
    std::cout << VectorDump<std::string> (crclinterface->crclwm.jointnames);
#if 0
    jointnames.push_back("joint_1")
    jointnames.push_back("joint_2")
    jointnames.push_back("joint_3")
    jointnames.push_back("joint_4")
    jointnames.push_back("joint_5")
    jointnames.push_back("joint_6")
    jointnames.push_back("joint_6-tool0")
#endif
}

int CCrcl2RosMsg::Action() {
    try {
        boost::mutex::scoped_lock lock(cncmutex);
        CAsioCrclSession *_pSession;

        /////////////////////////////////////////////////////////////////////////////////////////////
        // See if new CRCL commanded motion - if so, interpret as RCS command in session
        if (CAsioCrclSession::InMessages().SizeMsgQueue() > 0) {
            CrclMessage msg = CAsioCrclSession::InMessages().PopFrontMsgQueue();
            std::string crclcmd = boost::get<0>(msg);
            _pSession = boost::get<1>(msg);
            ROS_INFO("Crcl command: [%s] from %s:%d", crclcmd.c_str(),
                    _pSession->RemoteIP().c_str(),
                    _pSession->RemotePORT());

            Crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

            if (ret == Crcl::CANON_STATUSREPLY) {
                std::string sStatus = Crcl::CrclClientCmdInterface().GetStatusReply(&crclinterface->crclwm);
                _pSession->SyncWrite(sStatus);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        // interpret translated CRCL command. Commands in canonical form: standard units (mm, radians)
        // Note many CRCL commands are NOT translated into corresponding canoncial ROS commands
        if (RCS::cmds.SizeMsgQueue() > 0) {
            RCS::CanonCmd cc = RCS::cmds.PopFrontMsgQueue();
            nistcrcl::CrclCommandMsg rosmsg;
            // PUBLISH CRCL to ROS MESSAGE HERE _interpreter.ParseCommand(cc);

            enum crclcommand {
                noop = 0,
                initCanon = 1,
                endCanon = 2,
                actuatejoints = 3,
                moveto = 4,
                dwell = 5,
                message = 6,
                moveThroughTo = 7,
                setCoordinatedMotion = 8,
                stopMotion = 9,
                setEndEffector = 10,
                openToolChange = 11,
                closeToolChanger = 12,

            };
            rosmsg.crclcommand = noop;
            rosmsg.crclcommandnum = cc.CommandID();
            if (cc.cmd == RCS::CANON_MOVE_JOINT) {
                rosmsg.crclcommand = actuatejoints;
                rosmsg.joints = cc.joints;
                rosmsg.bCoordinated = cc.bCoordinated;
            } else if (cc.cmd == RCS::CANON_MOVE_TO) {
                rosmsg.crclcommand = moveto;
                rosmsg.finalpose.position.x = cc.pose.getOrigin().x();
                rosmsg.finalpose.position.y = cc.pose.getOrigin().y();
                rosmsg.finalpose.position.z = cc.pose.getOrigin().z();
                rosmsg.finalpose.orientation.x = cc.pose.getRotation().x();
                rosmsg.finalpose.orientation.y = cc.pose.getRotation().y();
                rosmsg.finalpose.orientation.z = cc.pose.getRotation().z();
                rosmsg.finalpose.orientation.w = cc.pose.getRotation().w();
            } else if (cc.cmd == RCS::CANON_STOP_MOTION) {
                rosmsg.crclcommand = stopMotion;
            } else if (cc.cmd == RCS::CANON_MOVE_THRU) {
                rosmsg.crclcommand = moveThroughTo;
            } else if (cc.cmd == RCS::CANON_DWELL) {
                rosmsg.crclcommand = dwell;
                rosmsg.dwell_seconds = cc.dwell;
            } else if (cc.cmd == RCS::CANON_SET_GRIPPER) {
                rosmsg.crclcommand = setEndEffector;
                rosmsg.eepercent = cc.gripperPos;
            }

            // publish ros message if found corresponding crcl command
            if (rosmsg.crclcommand != noop) {
                ROS_INFO("ROS command: [%d] ", rosmsg.crclcommand);
                crcl_pub.publish(rosmsg);
            }
        }
    } catch (std::exception & e) {
        std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "Exception in CController::Action() thread\n";
    }
    return 1;
}
