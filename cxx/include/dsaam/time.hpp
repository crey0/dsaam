#ifndef DSAAM_TIME_HPP
#define DSAAM_TIME_HPP
#include <ostream>
#include <limits>

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

  template<class T>
  struct TimeWithInf
  {
    TimeWithInf() : time() {}
    TimeWithInf(const T & time) : time(time) {} 
    static TimeWithInf inf(){ auto ret = TimeWithInf(true); ret._inf = true; return ret; }
    bool isinf() const { return _inf; };
    
    T time;
  private:
    bool _inf = false;
  };


  template<class T>
  inline bool operator <(const T &left, const TimeWithInf<T> &right)
  {
    return right.isinf() || left < right.time;
  }
  template<class T>
  inline bool operator>(const T &left, const TimeWithInf<T> &right){ return right < left; }
  template<class T>
  inline bool operator<=(const T &left, const TimeWithInf<T> &right){ return !(left > right); }
  template<class T>
  inline bool operator>=(const T &left, const TimeWithInf<T> &right){ return !(left > right); }
  template<class T>

  inline bool operator==(const T &left, const TimeWithInf<T> &right)
  {
    return !right.isinf() && left == right.time;
  }

  template<class T>
  inline bool operator!=(const Time &left, const TimeWithInf<T> &right)
  {
    return !(left == right);
  }


  template<class T>
  inline bool operator <(const TimeWithInf<T> &left, const T &right) { return !(right >= left);}
  template<class T>
  inline bool operator >(const TimeWithInf<T> &left, const T &right) { return !(right <= left);}
  template<class T>
  inline bool operator <=(const TimeWithInf<T> &left, const T &right) { return !(right > left);}
  template<class T>
  inline bool operator >=(const TimeWithInf<T> &left, const T &right) { return !(right > left);}
  template<class T>
  inline bool operator ==(const TimeWithInf<T> &left, const T &right) { return right == left;}
  template<class T>
  inline bool operator !=(const TimeWithInf<T> &left, const T &right) { return right != left;}
  template<class T>
  
  inline std::ostream& operator<<(std::ostream& os, const TimeWithInf<T>& right)
  {
    if (right.isinf())
      return os << "inf:inf";
    else
      return os << right.time;
  }

  template<class T, class T1>
  inline void min_assign(T & t, const T1& t1)
  {
    if(t1 < t) t = t1;
  }
}

#endif //DSAAM_TIME_HPP
