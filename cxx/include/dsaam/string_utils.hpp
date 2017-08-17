#ifndef DSAAM_STRING_UTILS
#define DSAAM_STRING_UTILS

#include<string>
#include<sstream>

namespace dsaam
{
  typedef std::string string;

  string to_string()
  {
    return "";
  }

  string to_string(const string & s)
  {
    return s;
  }

  template <typename T>
  string to_string(const T& object)
  {
    std::ostringstream ss;
    ss << object;
    return ss.str();
  }
  
  template<class... Args>
  string to_string(string s, Args... args)
  {
    return s + to_string(args...);
  }

  template<class T, class... Args>
  string to_string(T v, Args... args)
  {
    return to_string(v) + to_string(args...);
  }

}

#endif
