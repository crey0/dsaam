#ifndef DSAAM_TIME_HPP
#define DSAAM_TIME_HPP
#include <ostream>

namespace dsaam
{


  

  class Time
  {
  public:
    typedef long long int integer;
    static const constexpr integer NS_IN_SECOND = 1000000000;
    static const constexpr double  SECOND_IN_NS = 1e-9;

  public:
    
    Time(integer sec=0, integer nanos=0)
    {
      this->sec = sec;
      this->nanos = nanos;
    }

    Time(const Time &) = default;

    inline void operator=(const Time& other)
    {
      sec = other.sec;
      nanos = other.nanos;
    }

    explicit inline operator double() const 
    {
      return double(sec) + SECOND_IN_NS * double(nanos);
    }

  public:
    integer sec;
    integer nanos;
  };

  inline Time operator+(const Time &left, const Time &right)
  {
    Time::integer nanos = left.nanos % Time::NS_IN_SECOND + right.nanos;
    Time::integer carry = nanos / Time::NS_IN_SECOND + left.nanos / Time::NS_IN_SECOND \
	    + right.nanos / Time::NS_IN_SECOND;
    return Time(left.sec + right.sec + carry, nanos % Time::NS_IN_SECOND);
  }
  inline Time operator-(const Time &left, const Time &right)
  {
    Time::integer nanos = left.nanos % Time::NS_IN_SECOND - right.nanos;
    Time::integer carry = nanos / Time::NS_IN_SECOND + left.nanos / Time::NS_IN_SECOND\
      - right.nanos / Time::NS_IN_SECOND;
    return Time(left.sec - right.sec + carry, nanos % Time::NS_IN_SECOND);
  }
  inline Time operator*(const Time &left, Time::integer right)
  {
    Time::integer nanos = left.nanos % Time::NS_IN_SECOND * right;
    Time::integer carry = nanos / Time::NS_IN_SECOND + left.nanos / Time::NS_IN_SECOND;
    return Time(left.sec * right + carry, nanos % Time::NS_IN_SECOND);
  }
  inline Time operator+(Time::integer left, const Time &right){ return right + left; }
  inline Time operator*(Time::integer left, const Time &right){ return right * left; }

  inline bool operator<(const Time &left, const Time &right)
  {
    return left.sec < right.sec || (left.sec == right.sec && left.nanos < right.nanos);
  }
  inline bool operator>(const Time &left, const Time &right){ return right < left; }
  inline bool operator<=(const Time &left, const Time &right){ return !(left > right); }
  inline bool operator>=(const Time &left, const Time &right){ return !(left < right); }

  inline bool operator==(const Time &left, const Time &right)
  {
    return left.sec == right.sec && left.nanos == right.nanos;
  }
  inline bool operator!=(const Time &left, const Time &right){ return !(left == right); }

  inline std::ostream& operator<<(std::ostream& os, const Time& right)
  {
    return os << right.sec << ":" << right.nanos;
  }
  
}

#endif //DSAAM_TIME_HPP
