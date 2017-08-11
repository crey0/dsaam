#ifndef DSAAM_STRING_UTILS
#define DSAAM_STRING_UTILS

#include<string>

namespace dsaam
{
  typedef std::string string;

  string to_string()
  {
    return "";
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
