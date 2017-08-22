#ifndef DSAAM_COMMON_HPP
#define DSAAM_COMMON_HPP

#include<dsaam/exceptions.hpp>
#include<dsaam/time.hpp>
#include<dsaam/string_utils.hpp>
#include<functional>
#include<memory>
#include<vector>

namespace dsaam
{

  using std::string;

  class MessageBase;
  struct InFlow;
  struct OutFlow;
  struct Sink;
  
  typedef  std::function<void(const std::shared_ptr<MessageBase const>&, const Time &)>
  message_callback_type;

  typedef  std::function<void(const std::shared_ptr<MessageBase const> &)>
  send_callback_type;

  typedef  std::function<void(const Time &)>
  time_callback_type;
  static const message_callback_type EMPTY_MESSAGE_CALLBACK;
  static const send_callback_type EMPTY_SEND_CALLBACK;
  static const time_callback_type EMPTY_TIME_CALLBACK;

    class MessageBase
  {
  public:
    MessageBase(const Time &time) : time(time) {}
  public:
    const Time time;
  };
  
  typedef struct InFlow
  {
    InFlow(string name, Time dt,
	   const message_callback_type &callback = EMPTY_MESSAGE_CALLBACK,
	   const time_callback_type &time_callback = EMPTY_TIME_CALLBACK)
      : name(name), dt(dt), callback(callback), time_callback(time_callback) {}
    string name;
    Time dt;
    message_callback_type callback;
    time_callback_type time_callback;
  } InFlow;
  
  typedef struct Sink
  {
    Sink(string name, const send_callback_type &send_callback = EMPTY_SEND_CALLBACK)
      : name(name), send_callback(send_callback) {}
    string name;
    send_callback_type send_callback;
  } Sink;

  typedef struct OutFlow
  {
    OutFlow(string name, Time dt, std::vector<Sink> sinks)
      : name(name), dt(dt), sinks(sinks) {}
    string name;
    Time dt;
    std::vector<Sink> sinks;
  } OutFlow;

}

#endif
