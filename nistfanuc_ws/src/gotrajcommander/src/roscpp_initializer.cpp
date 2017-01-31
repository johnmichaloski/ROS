#include "roscpp_initializer.h"
#include "py_conversions.h"
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <memory>

static std::vector<std::string>& ROScppArgs()
{
  static std::vector<std::string> args;
  return args;
}

static std::string& ROScppNodeName()
{
  static std::string node_name("moveit_python_wrappers");
  return node_name;
}

void py_bindings_tools::roscpp_set_arguments(const std::string& node_name, boost::python::list& argv)
{
  ROScppNodeName() = node_name;
  ROScppArgs() = stringFromList(argv);
}

namespace
{
struct InitProxy
{
  InitProxy()
  {
    const std::vector<std::string>& args = ROScppArgs();
    int fake_argc = args.size();
    char** fake_argv = new char*[args.size()];
    for (std::size_t i = 0; i < args.size(); ++i)
      fake_argv[i] = strdup(args[i].c_str());

    ros::init(fake_argc, fake_argv, ROScppNodeName(),
              ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    for (int i = 0; i < fake_argc; ++i)
      delete[] fake_argv[i];
    delete[] fake_argv;
  }

  ~InitProxy()
  {
    if (ros::isInitialized() && !ros::isShuttingDown())
      ros::shutdown();
  }
};
}

static void roscpp_init_or_stop(bool init)
{
  // ensure we do not accidentally initialize ROS multiple times per process
  static boost::mutex lock;
  boost::mutex::scoped_lock slock(lock);

  // once per process, we start a spinner
  static bool once = true;
  static std::unique_ptr<InitProxy> proxy;
  static std::unique_ptr<ros::AsyncSpinner> spinner;

  // initialize only once
  if (once && init)
  {
    once = false;

    // if ROS (cpp) is not initialized, we initialize it
    if (!ros::isInitialized())
    {
      proxy.reset(new InitProxy());
      spinner.reset(new ros::AsyncSpinner(1));
      spinner->start();
    }
  }

  // shutdown if needed
  if (!init)
  {
    once = false;
    proxy.reset();
    spinner.reset();
  }
}

void py_bindings_tools::roscpp_init()
{
  roscpp_init_or_stop(true);
}

void py_bindings_tools::roscpp_init(const std::string& node_name, boost::python::list& argv)
{
  roscpp_set_arguments(node_name, argv);
  roscpp_init();
}

void py_bindings_tools::roscpp_init(boost::python::list& argv)
{
  ROScppArgs() = stringFromList(argv);
  roscpp_init();
}

void py_bindings_tools::roscpp_shutdown()
{
  roscpp_init_or_stop(false);
}

py_bindings_tools::ROScppInitializer::ROScppInitializer()
{
  roscpp_init();
}

py_bindings_tools::ROScppInitializer::ROScppInitializer(boost::python::list& argv)
{
  roscpp_init(argv);
}

py_bindings_tools::ROScppInitializer::ROScppInitializer(const std::string& node_name, boost::python::list& argv)
{
  roscpp_init(node_name, argv);
}
