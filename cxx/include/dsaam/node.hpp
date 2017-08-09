/***
node.hpp

***/
#ifndef DSAAM_NODE_HPP
#define DSAAM_NODE_HPP

#include<string>
#include<boost/heap/fibonacci_heap.hpp>
#include<dsaam/time.hpp>
#include<dsaam/queue.hpp>
#include<vector>
#include<list>
#include<functional>
#include<memory>

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

  template<class T>
  class MessageQueue : Queue<T>
  {
  public:
    MessageQueue(Time start_time, Time dt, unsigned int max_size)
      : Queue<T>(max_size), nextTime(start_time), dt(dt) {}

    T && pop()
    {
      T & m = Queue<T>::pop();
      assert(m->time == nextTime);
      nextTime = m->time + dt;
      return std::move(m);
    }

    const Time & nextAt() const
    {
      return nextTime; 
    }
      
  private:
    Time nextTime;
    Time dt;
  };

  template<typename T>
  bool operator<(const MessageQueue<T> &l, const MessageQueue<T> &r) { return l.nextAt() < r.nextAt();}
  template<typename T>
  bool operator>(const MessageQueue<T> &l, const MessageQueue<T> &r) { return r < l;}
  
  template<class T>
  class MessageFlowMultiplexer
  {
       
    class heap_data;
    typedef  boost::heap::fibonacci_heap<heap_data,
					 boost::heap::compare<std::greater<heap_data>>> heap_type;
    typedef std::allocator<MessageQueue<T>> MQAllocType;
    typedef std::allocator_traits<MQAllocType> MQAllocTraits;
    
    class heap_data
    {
    public:
      heap_data(unsigned int index, MessageQueue<T> & queue) : queue(queue) {}
      typename heap_type::handle_type handle;
      MessageQueue<T> & queue;

      bool operator>(const heap_data& r) const { return queue > r.queue;}
      bool operator<(const heap_data& r) const { return queue > r.queue;}
    };


  public:
    MessageFlowMultiplexer(std::list<InFlow> &inflows, const Time &time, unsigned int max_qsize):
      inflows(inflows), time(time), max_qsize(max_qsize), mqalloc(), heap()
    {
      this->queues = MQAllocTraits::allocate(mqalloc, inflows.size());
      int index = 0;
      for(InFlow flow : inflows)
      {
        MQAllocTraits::construct(mqalloc, &this->queues[index],
				 time, flow.dt, max_qsize);
	typename heap_type::handle_type h = heap.push(heap_data(index,
								this->queues[index]));
	(*h).handle = h;
	index++;
      } 
    }

    ~MessageFlowMultiplexer()
    {
      for(int i=0; i<inflows.size(); i++)
	{
	  MQAllocTraits::destroy(mqalloc, &queues[i]);
	}
      MQAllocTraits::deallocate(mqalloc, queues, inflows.size());
    }
  
  void push(string flow, MessageBase & message)
  {
    NULL;
  }

  MessageBase && pop()
  {
    heap_data & q = const_cast<heap_data &>(heap.top());
    auto m = q.queue.pop();
    heap.update(q.handle);
    return std::move(m);
  }

  Time nextTime()
  {
    return heap.top().queue.nextAt();
  }
    
  public:
    std::list<InFlow> inflows;
    Time time;
    unsigned int max_qsize;
    MQAllocType mqalloc;
    heap_type heap;
    MessageQueue<T> * queues;
   
    
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
    typedef std::shared_ptr<class BaseMessage> mpointer;
  public:      
    Node(string &name, Time &time, Time &dt, std::list<InFlow> &inflows,
	 std::list<OutFlow> &outflows, unsigned int max_qsize) :
      name(name), time(time), dt(dt),
      inflows(inflows, time, max_qsize), outflows()
    {
      for(OutFlow flow : outflows)
	{
	  this->outflows.push_back(OutMessageFlow(flow.name, time, flow.dt,
						  flow.sinks, max_qsize));
	}
    }

  public:
    string name;
    Time time;
    Time dt;
    MessageFlowMultiplexer<mpointer> inflows;
    std::list<OutMessageFlow> outflows;  
  };
    
}
#endif //DSAAM_NODE_HPP
