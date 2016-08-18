
#include "CsvLogging.h"
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
using namespace logging::trivial;

static const char *sStateEnums[] = {
	"CRCL_Done", "CRCL_Error", "CRCL_Working", "CRCL_Ready"
};
static const char * sCommand[] = {"Noop", "Init", "End", "MoveJoint", "MoveTo", "Dwell", "Message", "MoveThru", 
	"SetCoordinatedMotion", "Stop", "SetGripper", "OpenGripper", "CloseGripper"};
static  const char * sStatus[] = {"Done", "Error", "Working", "Paused", "Abort", "Waiting"};

static   src::severity_logger<severity_level> lg;


void InitLog() {
  typedef sinks::synchronous_sink< sinks::text_ostream_backend > text_sink;
  boost::shared_ptr< text_sink > sink1 = boost::make_shared< text_sink >();
  sink1->locked_backend()->add_stream(
         boost::make_shared< std::ofstream >("sign.log"));
  sink1->set_formatter (
         expr::format("[%1%]<%2%>(%3%): %4%")
         % expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
         % logging::trivial::severity
         % expr::attr<boost::log::attributes::current_thread_id::value_type >("ThreadID")
         % expr::smessage
         );
  logging::core::get()->add_sink(sink1);
  sink1->set_filter(expr::attr< severity_level >("Severity") >= warning);
  boost::shared_ptr< text_sink > sink2 = boost::make_shared< text_sink >();
  sink2->locked_backend()->add_stream(
         boost::make_shared< std::ofstream >("sign.csv"));
  sink2->set_formatter (
         expr::format("%1%,%2%,%3%")
         % expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
         % logging::trivial::severity
         % expr::smessage
         );
  logging::core::get()->add_sink(sink2);
  sink2->set_filter(expr::attr< severity_level >("Severity") < warning);
  logging::add_common_attributes();
  BOOST_LOG_SCOPED_THREAD_TAG("ThreadID", boost::this_thread::get_id());
}

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

	str << sCommand[status.crclcommand] << separator.c_str();
//	str << status.StatusID() << separator.c_str();
	str << sStateEnums[status.StatusID()] << separator.c_str();
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
// FIXME: publish status to crcl_status
// http://www.boost.org/doc/libs/1_60_0/libs/log/doc/html/log/tutorial/formatters.html
// http://csfreebird.blogspot.com/2013/12/use-boost-log-step-6.html <- This one!
void CsvLogging::MotionLogging(CanonWorldModel & status)
{
	if(!bHeader)
	{
		InitLog();
		BOOST_LOG_SEV(lg, trace) << DumpHeader(",", status.currentjoints.size());
		bHeader=true;
	}
 	/////////////////////////////////////////////////////////////////////////////////////////////
	// Save status to csv logging file?

		std::string sStatus = Dump(",") + "\n";

		if (lastlogstatus != sStatus) {
			BOOST_LOG_SEV(lg, trace) << sStatus;
		}
		lastlogstatus = sStatus;
}