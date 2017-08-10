#ifndef DSAAM_EXCEPTIONS
#define DSAAM_EXCEPTIONS
#include<string>
#include<stdexcept>

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
  
  inline void logic_except(bool test, const string & error)
  {
    if(!test)
      throw std::logic_error(error);
  }
}

#endif
