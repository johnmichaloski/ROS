#pragma once
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/expressions.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup.hpp>

#define LOG_TRACE  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::trace)
#define LOG_DEBUG  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::debug)
#define LOG_INFO  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::info)
#define LOG_WARN  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::warning)
#define LOG_ERROR BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::error)
#define LOG_FATAL BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::fatal)

extern std::string boostlogfile ;                       
extern boost::log::trivial::severity_level boostloglevel;

//Narrow-char thread-safe logger.
typedef boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger_t;

//declares a global logger with a custom initialization
BOOST_LOG_GLOBAL_LOGGER(my_logger, logger_t)
