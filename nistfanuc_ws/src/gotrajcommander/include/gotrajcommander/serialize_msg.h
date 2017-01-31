/* Author: Ioan Sucan */

#ifndef PY_BINDINGS_TOOLS_SERIALIZE_MSG_
#define PY_BINDINGS_TOOLS_SERIALIZE_MSG_

#include <ros/ros.h>


namespace py_bindings_tools
{
/** \brief Convert a ROS message to a string */
template <typename T>
std::string serializeMsg(const T& msg)
{
  // we use the fact char is same size as uint8_t;
  assert(sizeof(uint8_t) == sizeof(char));
  std::size_t size = ros::serialization::serializationLength(msg);
  std::string result(size, '\0');
  if (size)
  {
    // we convert the message into a string because that is easy to sent back & forth with Python
    // This is fine since C0x because &string[0] is guaranteed to point to a contiguous block of memory
    ros::serialization::OStream stream_arg(reinterpret_cast<uint8_t*>(&result[0]), size);
    ros::serialization::serialize(stream_arg, msg);
  }
  return result;
}

/** \brief Convert a string to a ROS message */
template <typename T>
void deserializeMsg(const std::string& data, T& msg)
{
  assert(sizeof(uint8_t) == sizeof(char));
  ros::serialization::IStream stream_arg(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&data[0])), data.size());
  ros::serialization::deserialize(stream_arg, msg);
}
}

#endif
