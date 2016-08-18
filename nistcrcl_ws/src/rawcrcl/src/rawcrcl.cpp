

// roslaunch nistcrcl crclserver.launch
// 129.6.32.176 64444
// You can test with nist java crcl tool.
// Assuming you have it installed: cd crcl4java; ./run.sh; ->  connect 

/***
Sample ROS output:
[ INFO] [1468518803.433983207]: Crcl listen: [127.0.0.1:64444]
Accept 127.0.0.1:47319

[ INFO] [1468518814.538768991]: Crcl command: [<?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>] from 127.0.0.1:47319


Problem is you need to send status to the client. And there can be multiple clients.


!!! Make sure you clean and source after you make messages, or rostopic will fail with message:
ERROR: Cannot load message class for [nistcrcl/crcl_command]. Are your messages built?

After connecting with javacrcl:
michalos@rufous:nistcrcl_ws> rostopic echo /crcl_command
xml: <?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>
ip: 127.0.0.1
port: 47548
---



*/
#include "rawcrcl.h"

#include <boost/format.hpp>
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
// THis is part of this package
#include "rawcrcl/CrclRawCommandMsg.h"


#include "AsioCrclServer.h"
#include "Globals.h"
#include "Setup.h"

ALogger Logger;
static ros::Publisher crcl_pub; /**< ros publisher information used for crcl command updates */
static ros::Subscriber crcl_sub ;


// myios declared as static in AsioCrclServer.h
CAsioCrclServer crclServer(myios);

CAsioCrclSession* pSession=NULL;

// This is very kludgy and assumes only 1 subscriber wants feedback
static void crclstatusCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Crcl status: [%s]", msg->data.c_str());
  if(pSession!=NULL)
     pSession->SyncWrite(msg->data);
}


int main(int argc, char** argv) {
    
      
        try {
        
        // Find path of executable
        std::string path(argv[0]);
        Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
        Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;
                
#if 0
       // This sets up some application name/value pairs: user, hostname
 		//SetupRosEnvironment - needs to go before ROS!
        SetupRosEnvironment("");
#endif
        
        // Initialize ROS
        ros::init(argc, argv, "nistcrcl");
        ros::NodeHandle nh;
        ros::Rate r(50);  // 10 times a second - 10Hz

        //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
        ros::AsyncSpinner spinner(1);
        spinner.start();
 
        // ROS config - parameter list - save for comparison later
        std::string params = ReadRosParams(nh);
        Globals.WriteFile(Globals.ExeDirectory + "rosconfig.txt", params);


#if 0
	// missing getPath while linking
        path = ros::package::getPath("nistcrcl");
        Globals._appproperties["nistcrcl"] = path;
#endif


       // Logger
        LogFile.Open(Globals.ExeDirectory + "logfile.log");
        LogFile.DebugLevel() = 5;
        LogFile.Timestamping() = true;
  
        // CRCl Communication handler - bundles xml into messages
         CAsioCrclServer::_bTrace = false;
        
        // This initializes the asio crcl socket listener, in theory N clients can connect. Only 1 tested.
	std::string crclip;
	int crclport=64444;
	nh.param<std::string>("crclip", crclip, "127.0.0.1");
	nh.param("crclport", crclport, 64444);
	ROS_INFO("Crcl listen: [%s:%d]", crclip.c_str(), crclport);
        //crclServer.Init("127.0.0.1", 64444, "Command GUI");
        crclServer.Init(crclip, crclport, "Command GUI");

        CAsioCrclServer::_bTrace=true;
        crclServer.Start();

       
	//crcl_pub = nh.advertise<std_msgs::String>("crcl_command", 10);
	//crcl_sub = nh.subscribe("crcl_status", 1000, crclstatusCallback);

	crcl_pub = nh.advertise<rawcrcl::CrclRawCommandMsg>("rawcrcl_command", 10);
	crcl_sub = nh.subscribe("rawcrcl_status", 1000, crclstatusCallback);

        spinner.stop();
        ros::spinOnce();
        do {
            myios.run_one();
	    if( CAsioCrclSession::InMessages().SizeMsgQueue() > 0)
		{
                CrclMessage crclmsg = CAsioCrclSession::InMessages().PopFrontMsgQueue();
                std::string crclcmd = boost::get<0>(crclmsg);
                pSession = boost::get<1>(crclmsg);
#if 0
 		std_msgs::String msg;
		msg.data =crclcmd;
 		ROS_INFO("Crcl command: [%s]", msg.data.c_str());
#endif
 		rawcrcl::CrclRawCommandMsg msg;
		msg.xml =crclcmd;
		msg.ip = pSession->RemoteIP();
		msg.port = pSession->RemotePORT();
 		ROS_INFO("Crcl command: [%s] from %s:%d", msg.xml.c_str(), msg.ip.c_str(),msg.port);
		crcl_pub.publish(msg);
		}
            ros::spinOnce();
            myios.run_one();
            r.sleep();
        } while(ros::ok());
        
        std::cout << "Cntrl C pressed \n"<< std::flush;
        
        crclServer.Stop();
        myios.stop();
    } catch (std::exception e) {
        Logger.Fatal(Logger.StrFormat("%s%s\n", "Abnormal exception end to  CRCL2Robot", e.what()));
    } catch (...) {
        Logger.Fatal("Abnormal exception end to  CRCL2Robot\n");
    }
    ros::shutdown();
    return 0;
}



