/***
node.hpp

***/
#ifndef DSAAM_NODE_HPP
#define DSAAM_NODE_HPP

#include<string>
#include<boost/heap/fibonacci_heap.hpp>
#include<dsaam/time.hpp>
#include<dsaam/queue.hpp>
#include<list>

namespace dsaam
{
  typedef std::string string;

  class MessageBase
  {
  public:
    MessageBase(const Time &time) : time(time) {}
  public:
    const Time time;
  };
  
  class MessageCallback
  {
  public:
    virtual void operator()(MessageBase m) = 0;
  };
  
  typedef struct InFlow
  {
    InFlow(string name, Time dt, MessageCallback &time_callback)
      : name(name), dt(dt), time_callback(time_callback) {}
      string name;
      Time dt;
      MessageCallback &time_callback;
  } InFlow;
  
  typedef struct Sink
  {
    Sink(string name, MessageCallback &send_callback)
      : name(name), send_callback(send_callback) {}
    string name;
    MessageCallback &send_callback;
  } Sink;

  typedef struct OutFlow
  {
    OutFlow(string name, Time dt, std::list<Sink> sinks)
      : name(name), dt(dt), sinks(sinks) {}
    string name;
    Time dt;
    std::list<Sink> sinks;
  } OutFlow;
    
  class MessageFlowMultiplexer
  {
  public:
    MessageFlowMultiplexer(std::list<InFlow> &inflows, const Time &time, unsigned int max_qsize):
      time(time), max_qsize(max_qsize)
    {
      for(InFlow flow : inflows)
      {
        this->inflows.push_back(Queue<MessageBase>(max_qsize));
      } 
    }
  
  void push(string flow, MessageBase & message)
  {
    NULL;
  }

  MessageBase & pop()
  {
    return NULL;
  }

  Time nextTime()
  {
  NULL;
}
    
public:
  Time time;
  std::list<Queue<MessageBase>> inflows;
  unsigned int max_qsize;
    
};

  class OutMessageFlow
  {
  public:
    OutMessageFlow(string name, Time start_time, Time dt, std::list<Sink> &sinks,
		   unsigned int max_qsize)
      : name(name), time(start_time), dt(dt), sinks(sinks) {}
  public:
    const string name;
  private:
    Time time;
    Time dt;
    std::list<Sink> sinks;
  };

  class Node
  {
  public:      
    Node(string &name, Time &time, Time &dt, void *inflows, std::list<OutFlow> outflows, unsigned int max_qsize) :
      name(name), time(time), dt(dt), inflows(inflows, time, max_qsize), outflows(outflows.size())
    {
      for(OutFlow flow : outflows)
	{
	  this->outflows.push_back(OutMessageFlow(flow.name, time, flow.dt, flow.sinks, max_qsize));
	}
    }

  public:
    string name;
    Time time;
    Time dt;
    MessageFlowMultiplexer inflows;
    std::list<OutMessageFlow> outflows;  
  };
    
}
#endif //DSAAM_NODE_HPP
