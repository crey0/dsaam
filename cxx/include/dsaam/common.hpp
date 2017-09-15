#ifndef DSAAM_COMMON_HPP
#define DSAAM_COMMON_HPP

#include<dsaam/exceptions.hpp>
#include<dsaam/string_utils.hpp>
#include<functional>
#include<memory>
#include<vector>

namespace dsaam
{

  using std::string;

  template<class M, class T, template <class> class F> struct InFlow;
  template<class M, class T, template <class> class F, class S> struct OutFlow;
  struct Sink;

  template<class M, class T, template <class> class F>
  using  message_callback_type =  F<void(const M&, const T &)>;

  template<class M, template <class> class F>
  using send_callback_type = F<void(const M &)>;

  template<class T, template <class> class F>
  using time_callback_type = F<void(const T &)>;

  template<class T>
  struct empty
  {
    static constexpr const T value()
    {
      return T();
    }
  };


  template<class M, class T, template <class> class F>
  struct InFlow
  {
    InFlow() = default;
    InFlow(string name, const T& time, const T& dt, size_t qsize = 0,
	   const message_callback_type<M,T,F> &callback = empty<message_callback_type<M,T,F>>::value(),
	   const time_callback_type<T,F> &time_callback = empty<time_callback_type<T,F>>::value())
      : name(name), time(time), dt(dt), qsize(qsize), callback(callback), time_callback(time_callback) {}
    string name;
    T time;
    T dt;
    size_t qsize;
    message_callback_type<M, T, F> callback;
    time_callback_type<T, F> time_callback;
  };
  
  struct Sink
  {
    Sink() = default;
    Sink(string name)
      : name(name) {}
    string name;
  };

  template<class M, class T, template <class> class F, class S = Sink>
  struct OutFlow
  {
    OutFlow() = default;
    OutFlow(string name, const T& time, const T& dt, const std::vector<S>& sinks = {}, size_t qsize = 0)
      : name(name), time(time), dt(dt), qsize(qsize), send(), sinks(sinks) {}

    void setup_sink(S &&sink)
    {
      sinks.emplace_back(std::move(sink));
    }
    
    string name;
    T time;
    T dt;
    size_t qsize;
    send_callback_type<M, F> send;
    std::vector<S> sinks;
  };

}

#endif
