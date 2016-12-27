


// From: http://pastebin.com/e3qDQ8tT
#include <boost/log/attributes.hpp>
#include <boost/log/common.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/formatter_parser.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <fstream>

namespace logging = boost::log;
namespace attrs = boost::log::attributes;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;

enum sign_severity_level {
  trace,
  debug,
  info,
  warning,
  error,
  fatal,
  report
};

BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(
    my_logger, src::severity_logger_mt<sign_severity_level>
);

/**
Sample use:
int main() {
  Manager manager;
  auto log = my_logger::get();

  manager.set_log_file("log_1.txt");
  BOOST_LOG_SEV(log, error) << "This will go to log_1";
  BOOST_LOG_SEV(log, error) << "And this one";

  manager.set_log_file("log_2.txt");
  BOOST_LOG_SEV(log, error) << "This will go to log_2";

  manager.set_log_file("log_3.txt");
  BOOST_LOG_SEV(log, error) << "This will go to log_3";
}
*/
class LogManager {
 public:
  using Backend = sinks::text_ostream_backend;
  using TextSink = sinks::synchronous_sink<Backend>;

  LogManager() {
    backend_ = boost::make_shared<Backend>();
    backend_->auto_flush(true);

    boost::shared_ptr<TextSink> sink(new TextSink(backend_));
    sink->set_formatter(
        expr::format("[%1%]<%2%>(%3%): %4%") %
            expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S") %
            expr::attr < sign_severity_level > ("Severity") %
            expr::attr < attrs::current_thread_id::value_type > ("ThreadID") %
            expr::smessage
    );
    sink->set_filter(expr::attr<sign_severity_level>("Severity") >= warning);
    logging::core::get()->add_sink(sink);

    logging::add_common_attributes();
    logging::core::get()->add_global_attribute(
        "ThreadID", attrs::current_thread_id()
    );
  }

  void set_log_file(const char* filename) {
    backend_->remove_stream(current_stream_);
    current_stream_.reset(new std::ofstream(filename));
    backend_->add_stream(current_stream_);
  }

 private:
  boost::shared_ptr<Backend> backend_;
  boost::shared_ptr<std::ostream> current_stream_;
};


