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

  template<class T, class U, class... Args>
  inline string to_string(T v, U v2, Args... args)
  {
    return dsaam::to_string(v) + dsaam::to_string(v2, args...);
  }

}

#endif
