/* Author: Ioan Sucan */

#ifndef PY_BINDINGS_TOOLS_PY_CONVERSIONS_
#define PY_BINDINGS_TOOLS_PY_CONVERSIONS_

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>


namespace py_bindings_tools
{
template <typename T>
std::vector<T> typeFromList(const boost::python::object& values)
{
  boost::python::stl_input_iterator<T> begin(values), end;
  std::vector<T> v;
  v.assign(begin, end);
  return v;
}

template <typename T>
boost::python::list listFromType(const std::vector<T>& v)
{
  boost::python::list l;
  for (std::size_t i = 0; i < v.size(); ++i)
    l.append(v[i]);
  return l;
}

template <typename T>
boost::python::dict dictFromType(const std::map<std::string, T>& v)
{
  boost::python::dict d;
  for (typename std::map<std::string, T>::const_iterator it = v.begin(); it != v.end(); ++it)
    d[it->first] = it->second;
  return d;
}

std::vector<double> doubleFromList(const boost::python::object& values)
{
  return typeFromList<double>(values);
}

std::vector<std::string> stringFromList(const boost::python::object& values)
{
  return typeFromList<std::string>(values);
}

boost::python::list listFromDouble(const std::vector<double>& v)
{
  return listFromType<double>(v);
}

boost::python::list listFromString(const std::vector<std::string>& v)
{
  return listFromType<std::string>(v);
}
}


#endif
