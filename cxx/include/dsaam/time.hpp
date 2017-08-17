#ifndef DSAAM_TIME_HPP
#define DSAAM_TIME_HPP
#include <ostream>

namespace dsaam
{

  typedef long long int integer;

  integer NS_IN_SECOND = 1000000000;
  double  SECOND_IN_NS = 1e-9;

  class Time
  {
  public:
    
    Time(integer sec=0, integer nanos=0)
    {
      this->sec = sec;
      this->nanos = nanos;
    }

    Time(const Time &) = default;

    void operator=(const Time& other)
    {
      sec = other.sec;
      nanos = other.nanos;
    }

    explicit operator double() const 
    {
      return double(sec) + SECOND_IN_NS * double(nanos);
    }

  public:
    integer sec;
    integer nanos;
  };

  Time operator+(const Time &left, const Time &right)
  {
    integer nanos = left.nanos % NS_IN_SECOND + right.nanos;
    integer carry = nanos / NS_IN_SECOND + left.nanos / NS_IN_SECOND + right.nanos / NS_IN_SECOND;
    return Time(left.sec + right.sec + carry, nanos % NS_IN_SECOND);
  }
  Time operator-(const Time &left, const Time &right)
  {
    integer nanos = left.nanos % NS_IN_SECOND - right.nanos;
    integer carry = nanos / NS_IN_SECOND + left.nanos / NS_IN_SECOND - right.nanos / NS_IN_SECOND;
    return Time(left.sec - right.sec + carry, nanos % NS_IN_SECOND);
  }
  Time operator*(const Time &left, integer right)
  {
    integer nanos = left.nanos % NS_IN_SECOND * right;
    integer carry = nanos / NS_IN_SECOND + left.nanos / NS_IN_SECOND;
    return Time(left.sec * right + carry, nanos % NS_IN_SECOND);
  }
  Time operator+(integer left, const Time &right){ return right + left; }
  Time operator*(integer left, const Time &right){ return right * left; }

  bool operator<(const Time &left, const Time &right)
  {
    return left.sec < right.sec || (left.sec == right.sec && left.nanos < right.nanos);
  }
  bool operator>(const Time &left, const Time &right){ return right < left; }
  bool operator<=(const Time &left, const Time &right){ return !(left > right); }
  bool operator>=(const Time &left, const Time &right){ return !(left < right); }

  bool operator==(const Time &left, const Time &right)
  {
    return left.sec == right.sec && left.nanos == right.nanos;
  }
  bool operator!=(const Time &left, const Time &right){ return !(left == right); }

  std::ostream& operator<<(std::ostream& os, const Time& right)
  {
    return os << right.sec << ":" << right.nanos;
  }
  
}

#endif //DSAAM_TIME_HPP
