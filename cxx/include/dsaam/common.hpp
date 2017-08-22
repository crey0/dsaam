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

  template<class M, class T> struct InFlow;
  template<class M, class T> struct OutFlow;
  template<class M> struct Sink;

  template<class M, class T>
  using  message_callback_type =  std::function<void(const M&, const T &)>;

  template<class M>
  using send_callback_type = std::function<void(const M &)>;

  template<class T>
  using time_callback_type = std::function<void(const T &)>;

  template<class T>
  struct empty
  {
    static constexpr const T value()
    {
      return T();
    }
  };


  template<class M, class T>
  struct InFlow
  {
    InFlow(string name, T dt,
	   const message_callback_type<M,T> &callback = empty<message_callback_type<M,T>>::value(),
	   const time_callback_type<T> &time_callback = empty<time_callback_type<T>>::value())
      : name(name), dt(dt), callback(callback), time_callback(time_callback) {}
    string name;
    T dt;
    message_callback_type<M, T> callback;
    time_callback_type<T> time_callback;
  };
  
  template<class M>
  struct Sink
  {
    Sink(string name, const send_callback_type<M> &send_callback = empty<send_callback_type<M>>::value())
      : name(name), send_callback(send_callback) {}
    string name;
    send_callback_type<M> send_callback;
  };

  template<class M, class T>
  struct OutFlow
  {
    OutFlow(string name, T dt, std::vector<Sink<M>> sinks)
      : name(name), dt(dt), sinks(sinks) {}
    string name;
    T dt;
    std::vector<Sink<M>> sinks;
  };

}

#endif
