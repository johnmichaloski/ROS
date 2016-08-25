
#pragma once
#include <boost/shared_ptr.hpp>
#include <list>

#include <ros/ros.h>
#include <ros/package.h>

#include "AsioCrclServer.h"
#include "RCSThreadTemplate.h"
#include "RCSMsgQueueThread.h"

#include "CrclInterface.h"
#include "nistcrcl/CrclCommandMsg.h"
#include "nistcrcl/CrclStatusMsg.h"

#include "RCS.h"
#include "crcl.h"

namespace RCS
{
extern  CMessageQueue<RCS::CanonCmd> cmds;
extern CanonWorldModel wm;  // motion related parameters max vel,acc
// not sure why this cant be part of crclwm

};

class CCrcl2RosMsg : public CAsioMessageQueueThread 
{
public:
    CCrcl2RosMsg(ros::NodeHandle &nh) : _nh(nh)
    {

    }
    /*!
         *\brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
         * cmds queue, interprets into robot command messages.
         */
    virtual int Action();

    /*!
         *\brief Initialization routine for the controller..
         */
    virtual void Init();
    void StatusCallback(const nistcrcl::CrclStatusMsg::ConstPtr& statusmsg);
    ros::NodeHandle &_nh;
    ros::Publisher crcl_pub; /**< ros publisher information used for crcl command updates */
    ros::Subscriber crcl_sub ;
    static boost::mutex cncmutex; /**< mutex for thread safe access to RobotProgram commands  */
    boost::shared_ptr<Crcl::CrclDelegateInterface> crclinterface;
    std::vector<std::string> jointnames;

};


