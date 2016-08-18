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


#include "BLogging.h"

namespace attrs   = boost::log::attributes;
namespace expr    = boost::log::expressions;
namespace logging = boost::log;

//Defines a global logger initialization routine
BOOST_LOG_GLOBAL_LOGGER_INIT(my_logger, logger_t)
{
    logger_t lg;

    logging::add_common_attributes();

    logging::add_file_log(
            boost::log::keywords::file_name = SYS_LOGFILE,
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
        logging::trivial::severity >= logging::trivial::info
    );

    return lg;
}