
#include "CsvLogging.h"
#include "BLogging.h"
#include <fstream>

static const char *sStateEnums[] = {
	"CRCL_Done", "CRCL_Error", "CRCL_Working", "CRCL_Ready"
};
static const char * sCommand[] = {"Noop", "Init", "End", "MoveJoint", "MoveTo", "Dwell", "Message", "MoveThru", 
	"SetCoordinatedMotion", "Stop", "SetGripper", "OpenGripper", "CloseGripper"};
static  const char * sStatus[] = {"Done", "Error", "Working", "Paused", "Abort", "Waiting"};
#define S1(x) #x
#define S2(x) S1(x)
#define LOCATION __func__ " : " S2(__LINE__)
#define LOG_ONCE(X) {static int F=0; if(!F) { X;F=1; } }
namespace RCS
{

	std::string CsvLogging::DumpHeader(std::string separator, size_t jointnum) {
		std::stringstream str;

		const char *rpyfields[] = {"Command", "State", "Pose-X", "Pose-Y", "Pose-Z", "Roll", "Pitch", "Yaw"};

		for (size_t i = 0; i < sizeof ( rpyfields) / sizeof ( rpyfields[0]); i++) {
			str << rpyfields[i] << separator.c_str();
		}

		for (size_t i = 0; i < jointnum; i++) {
			str << ((i > 0) ? separator.c_str() : "") << "Joint" << i;
		}
		return str.str();
	}

	std::string CsvLogging::Dump(std::string separator, CanonWorldModel & status) {
		std::stringstream str;
		str.precision(4);
		CanonCmdType echo_cmd; /**<  copy of current command type */
		CanonStatusType echo_status; /**<  copy of current status type */

		str << sCommand[status.echocmd.crclcommand] << separator.c_str();
		//	str << status.StatusID() << separator.c_str();
		str << sStateEnums[status.echo_status] << separator.c_str();
		RCS::Pose pose = status.currentpose;
		str << pose.getOrigin().x() << separator.c_str();
		str << pose.getOrigin().y() << separator.c_str();
		str << pose.getOrigin().z() << separator.c_str();

		double roll, pitch, yaw;
		getRPY(pose, roll, pitch, yaw);
		//tf::Matrix3x3 rot = pose.getBasis();
		//rot.getRPY(roll, pitch, yaw);
		//pose.rotation.getRPY(roll, pitch, yaw);
		str << roll << separator.c_str();
		str << pitch << separator.c_str();
		str << yaw;

		sensor_msgs::JointState joints = status.currentjoints;
		for (size_t i = 0; i < joints.position.size(); i++) {
			str << separator.c_str() << joints.position[i];
		}
		return str.str();
	}
	void CsvLogging::Open(std::string filepath) 
	{  
		fs.open (filepath,  std::fstream::out); 
		if(!fs)
			LOG_ONCE(LOG_DEBUG << "CsvLogging::Open file opened for logging failed");
	}
	void CsvLogging::MotionLogging(CanonWorldModel & status)
	{
		if(!fs.is_open())
		{
			LOG_ONCE(LOG_DEBUG << "CsvLogging::MotionLogging no file opened for logging");
			return;
		}
		if(!bHeader)
		{
			fs << DumpHeader(",", status.currentjoints.position.size());
			bHeader=true;
		}
		/////////////////////////////////////////////////////////////////////////////////////////////
		// Save status to csv logging file?

		std::string sStatus = Dump(",",status) + "\n";

		if (lastlogstatus != sStatus) {
			fs << sStatus;
		}
		lastlogstatus = sStatus;
	}
}
