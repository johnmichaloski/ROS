

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

/**
http://stackoverflow.com/questions/20086754/how-to-use-boost-log-from-multiple-files-with-gloa
#include "Logging.h"

int main(int argc, char **argv)
{
    INFO << "Program started";

    return 0;
}
My build settings

AM_LDFLAGS += -lboost_system -lboost_thread -lpthread
AM_LDFLAGS += -DBOOST_LOG_DYN_LINK -lboost_log_setup -lboost_log
AM_CXXFLAGS += -std=c++11 -DBOOST_LOG_DYN_LINK

*/


#include "NIST/Boost.h"
#include "Globals.h"
// http://pastebin.com/e3qDQ8tT

namespace attrs   = boost::log::attributes;
namespace expr    = boost::log::expressions;
namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

std::string boostlogfile = "/var/log/example.log";
boost::log::trivial::severity_level boostloglevel=logging::trivial::info;

boost::shared_ptr<sinks::text_ostream_backend> backend_;
boost::shared_ptr<std::ostream> current_stream_;
  
  
//Defines a global logger initialization routine
BOOST_LOG_GLOBAL_LOGGER_INIT(my_logger, logger_t)
{
    logger_t lg;

    logging::add_common_attributes();

    logging::add_file_log(
            boost::log::keywords::file_name = boostlogfile.c_str(),
            boost::log::keywords::format = (
                    expr::stream << expr::format_date_time<     boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
                    << " [" << expr::attr<     boost::log::trivial::severity_level >("Severity") << "]: "
                    << expr::smessage
            )
    );

    logging::add_console_log(
            std::cout,
            boost::log::keywords::format = (
                    expr::stream << expr::format_date_time<     boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S")
                    << " [" << expr::attr<     boost::log::trivial::severity_level >("Severity") << "]: "
                    << expr::smessage
            )
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );
    backend_ = boost::make_shared<sinks::text_ostream_backend>();
    backend_->auto_flush(true);
    return lg;
}

void set_log_file(const char* filename) {
    std::string file = Globals.ExeDirectory + filename;
    backend_->remove_stream(current_stream_);
    current_stream_.reset(new std::ofstream(file.c_str()));
    backend_->add_stream(current_stream_);
}
