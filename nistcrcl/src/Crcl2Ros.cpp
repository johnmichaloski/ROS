


#include "Crcl2Ros.h"

namespace RCS
{
CMessageQueue<RCS::CanonCmd> cmds;
CanonWorldModel wm;
}
boost::mutex CCrcl2RosMsg::cncmutex;
void CCrcl2RosMsg::Init() {

    Name() = "CCrcl2RosMsg";
    // Controller instantiatio of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<Crcl::CrclDelegateInterface>(
            new Crcl::CrclDelegateInterface());
    crclinterface->SetAngleUnits("DEGREE");
    crcl_pub = _nh.advertise<nistcrcl::CrclCommandMsg>("crcl_command", 10);
//    crcl_sub = _nh.subscribe("crcl_status", 1000, crclstatusCallback);
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
                     _pSession->RemoteIP ().c_str(),
                     _pSession->RemotePORT ());

            Crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

            if (ret == Crcl::CANON_STATUSREPLY) {
                std::string sStatus = Crcl::CrclClientCmdInterface().GetStatusReply(&crclinterface->crclwm);
                _pSession->SyncWrite(sStatus);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        // interpret translated CRCL command. Commands in canonical form: standard units (mm, radians)
        if (RCS::cmds.SizeMsgQueue() > 0) {
            RCS::CanonCmd cc = RCS::cmds.PopFrontMsgQueue();
            nistcrcl::CrclCommandMsg rosmsg;
            // PUBLISH CRCL to ROS MESSAGE HERE _interpreter.ParseCommand(cc);
            enum crclcommand
            {
                noop=0,
                initCanon=1,
                endCanon=2,
                actuatejoints=3,
                moveto=4,
                dwell=5,
                message=6,
                moveThroughTo=7,
                setCoordinatedMotion=8,
                stopMotion=9,
                setEndEffector=10,
                openToolChange=11,
                closeToolChanger=12,

            };
            rosmsg.crclcommand=noop;

            if (cc.cmd == RCS::CANON_MOVE_JOINT)
            {
                rosmsg.crclcommand=actuatejoints;
                 rosmsg.joints=cc.joints;
            }
            else if (cc.cmd == RCS::CANON_MOVE_TO)
            {
                rosmsg.crclcommand=moveto;
                rosmsg.finalpose.position.x=cc.pose.getOrigin().x();
                rosmsg.finalpose.position.y=cc.pose.getOrigin().y();
                rosmsg.finalpose.position.z=cc.pose.getOrigin().z();
                rosmsg.finalpose.orientation.x=cc.pose.getRotation().x();
                rosmsg.finalpose.orientation.y=cc.pose.getRotation().y();
                rosmsg.finalpose.orientation.z=cc.pose.getRotation().z() ;
                rosmsg.finalpose.orientation.w=cc.pose.getRotation().w();
            }
            else if (cc.cmd == RCS::CANON_STOP_MOTION)
            {
                rosmsg.crclcommand=stopMotion;
            }
            else if (cc.cmd == RCS::CANON_MOVE_THRU)
            {
                rosmsg.crclcommand=moveThroughTo;
            }
            else if (cc.cmd == RCS::CANON_DWELL)
            {
                rosmsg.crclcommand=dwell;
                rosmsg.dwell_seconds = cc.dwell;
            }
            else if (cc.cmd == RCS::CANON_SET_GRIPPER)
            {
                rosmsg.crclcommand=setEndEffector;
                rosmsg.eepercent = cc.gripperPos;
            }

            // publish ros message if found corresponding crcl command
            if(rosmsg.crclcommand!=noop)
            {
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
