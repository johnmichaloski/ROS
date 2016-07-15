
#pragma once
#include <boost/shared_ptr.hpp>
#include <list>

#include <ros/ros.h>
#include <ros/package.h>

#include "AsioCrclServer.h"
#include "RCSThreadTemplate.h"
#include "CrclInterface.h"
#include "nistcrcl/CrclCommandMsg.h"

#include "RCS.h"
#include "crcl.h"

namespace RCS
{
extern  CMessageQueue<RCS::CanonCmd> cmds;
extern CanonWorldModel wm;

};

class CCrcl2RosMsg : public RCS::Thread
{
public:
    CCrcl2RosMsg(ros::NodeHandle &nh , double cycletime=100.0) : _nh(nh), RCS::Thread(cycletime)
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

    ros::NodeHandle &_nh;
    ros::Publisher crcl_pub; /**< ros publisher information used for crcl command updates */
    ros::Subscriber crcl_sub ;
    static boost::mutex cncmutex; /**< mutex for thread safe access to RobotProgram commands  */
    boost::shared_ptr<Crcl::CrclDelegateInterface> crclinterface;
};


