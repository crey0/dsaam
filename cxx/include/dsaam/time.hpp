/*
Copyright © 2018 CNRS
All rights reserved.

@author Christophe Reymann

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
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
