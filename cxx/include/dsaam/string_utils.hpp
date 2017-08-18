#ifndef DSAAM_STRING_UTILS
#define DSAAM_STRING_UTILS

#include<string>
#include<sstream>

namespace dsaam
{
  typedef std::string string;

  inline string to_string()
  {
    return "";
  }

  inline string to_string(const string & s)
  {
    return s;
  }

  template <typename T>
  inline string to_string(const T& object)
  {
    std::ostringstream ss;
    ss << object;
    return ss.str();
  }
  
  template<class... Args>
  inline string to_string(string s, Args... args)
  {
    return s + to_string(args...);
  }

  template<class T, class... Args>
  inline string to_string(T v, Args... args)
  {
    return to_string(v) + to_string(args...);
  }

}

#endif
