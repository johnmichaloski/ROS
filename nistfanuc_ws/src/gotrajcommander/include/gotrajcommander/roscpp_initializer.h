#ifndef PY_BINDINGS_TOOLS_ROSCPP_INITIALIZER_
#define PY_BINDINGS_TOOLS_ROSCPP_INITIALIZER_

#include <boost/python.hpp>
#include <string>


/** \brief Tools for creating python bindings for MoveIt */
namespace py_bindings_tools
{
/** \brief The constructor of this class ensures that ros::init() has
    been called.  Thread safety and multiple initialization is
    properly handled. When the process terminates, ros::shotdown() is
    also called, if needed. */
class ROScppInitializer
{
public:
  ROScppInitializer();
  ROScppInitializer(boost::python::list& argv);
  ROScppInitializer(const std::string& node_name, boost::python::list& argv);
};

/** \brief This function can be used to specify the ROS command line arguments for the internal ROScpp instance;
    Usually this function would also be exposed in the py module that uses ROScppInitializer. */
void roscpp_set_arguments(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(boost::python::list& argv);

/** \brief Initialize ROScpp with default command line args */
void roscpp_init();

void roscpp_shutdown();
}

