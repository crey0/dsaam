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
#include<deque>
#include<list>
#include<functional>
#include<memory>
#include<mutex>
#include <exception>

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

  typedef  void(*message_callback)(const std::shared_ptr<MessageBase const>, const Time &);
  typedef  void(*send_callback)(const std::shared_ptr<MessageBase const>);
  typedef  void(*time_callback)(const Time &);
  
  typedef struct InFlow
  {
    InFlow(string name, Time dt,
	   message_callback &callback, time_callback &time_callback)
      : name(name), dt(dt), callback(callback), time_callback(time_callback) {}
    string name;
    Time dt;
    message_callback &callback;
    time_callback &time_callback;
  } InFlow;
  
  typedef struct Sink
  {
    Sink(string name, send_callback &send_callback)
      : name(name), send_callback(send_callback) {}
    string name;
    send_callback &send_callback;
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
  class MessageQueue : public Queue<T>
  {
  public:
    MessageQueue(Time start_time, Time dt, unsigned int max_size)
      : Queue<T>(max_size), nextTime(start_time), dt(dt) {}

    T && pop()
    {
      T && m = Queue<T>::pop();
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
  bool operator<(const MessageQueue<T> &l, const MessageQueue<T> &r)
  { return l.nextAt() < r.nextAt();}

  template<typename T>
  bool operator>(const MessageQueue<T> &l, const MessageQueue<T> &r)
  { return r < l;}
  
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
      heap_data(InFlow &flow, MessageQueue<T> & queue) : flow(flow), queue(queue) {}
      InFlow &flow;
      MessageQueue<T> & queue;
      typename heap_type::handle_type handle;

      bool operator>(const heap_data& r) const { return queue > r.queue;}
      bool operator<(const heap_data& r) const { return queue > r.queue;}
    };


  public:
    MessageFlowMultiplexer(std::list<InFlow> &inflows, const Time &time, unsigned int max_qsize):
      inflows(inflows), time(time), max_qsize(max_qsize), mqalloc(), heap()
    {
      this->queues = MQAllocTraits::allocate(mqalloc, inflows.size());
      int index = 0;
      for(InFlow & flow : this->inflows)
	{
	  MQAllocTraits::construct(mqalloc, &this->queues[index],
				   time, flow.dt, max_qsize);
	  typename heap_type::handle_type h = heap.push(heap_data(flow,
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

    unsigned int flow_index(const string & name)
    {
      unsigned int i = 0;
      for(auto s : inflows)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No inflow " + name + " defined on this node");
 
    }
  
    void push(unsigned int flow_index, T && message)
    {
      queues[flow_index].push(std::move(message));
    }

    void next()
    {
      heap_data & q = const_cast<heap_data &>(heap.top());
      auto m = q.queue.pop();
      heap.update(q.handle);
      q.flow.time_callback(nextTime());
      q.flow.callback(std::move(m), nextTime());
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

  template<class T>
  class OutMessageFlow
  {
    typedef boost::heap::fibonacci_heap<int> heap_type;
  public:
    OutMessageFlow(string name, Time start_time, Time dt, std::list<Sink> &sinks,
		   unsigned int max_qsize)
      :  name(name), max_qsize(max_qsize), time(start_time), dt(dt), sinks(sinks),
	 heap(), heap_handles() {}


    unsigned int subscriber_index(const string & name)
    {
      unsigned int i = 0;
      for(auto s : sinks)
	{
	  if (s.name == name) return i;
	  i++;
	}
      throw  std::domain_error("No sink " + name + " defined on outflow " + this->name);
 
    }


    void time_callback(unsigned int subscriber, const Time &t)
    {
      int top = max_qsize;
      {
	std::lock_guard<std::mutex>(this->m);
      
	typename heap_type::handle_type h = heap_handles[subscriber];
	*h -= 1;
	heap.update(h);
	top = heap.top();
      }
      if (top<max_qsize)
	cv.notify_one();
    }
    
    void send(T message)
    {
      assert(message->time == time + dt);

      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [this]{return heap.top() > 0;});
      
      unsigned int index = 0;
      for(Sink s : sinks)
	{
	  if(s.send_callback != nullptr) s.send_callback(message);
	  typename heap_type::handle_type h=heap_handles[index++];
	  (*h) += 1;
	  heap.update(h);
	}

      lk.unlock();
      cv.notify_one();
    }

  public:
    const string name;

  private:
    unsigned int max_qsize;
    Time time;
    Time dt;
    std::list<Sink> sinks;
    heap_type heap;
    std::vector<heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };

  class Node
  {
    typedef std::shared_ptr<class MessageBase> mpointer;
  public:      
    Node(string &name, Time &time, Time &dt, std::list<InFlow> &inflows,
	 std::list<OutFlow> &outflows, unsigned int max_qsize) :
      name(name), _time(time), _dt(dt),
      inflows(inflows, time, max_qsize), outflows()
    {
      for(OutFlow flow : outflows)
	{
	  this->outflows.emplace_back(flow.name, _time, flow.dt,
				      flow.sinks, max_qsize);
	}
    }

    void next()
    {
      this->inflows.next();
    }

    //TODO: how to implement assert nextTime >= message.time ?
    std::function<void (mpointer)> send_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&Node::_send, this, &outflows[flow], _1);
    }
    
    std::function<void (mpointer)> send_callback(const string & flow)
    {
      return send_callback(_out_flow_index(flow));
    }

    std::function<void (const Time &)> time_callback(unsigned int flow, unsigned int sink)
    {
      using std::placeholders::_1;
      return std::bind(&OutMessageFlow<mpointer>::time_callback, &outflows[flow], sink, _1);
    }

    std::function<void (const Time &)> time_callback(const string &flow, const string &sink)
    {
      unsigned int iflow = _out_flow_index(flow);
      return time_callback(iflow, _out_flow_sink_index(iflow, sink));
    }
    std::function<void (mpointer)> message_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&MessageFlowMultiplexer<mpointer>::push, inflows, flow, _1);
    }

    std::function<void (mpointer)> message_callback(const string &flow)
    {
      return message_callback(_in_flow_index(flow));
    }

    void step(const Time & t)
    {
      assert(t >= _time);
      _time = t;
    }

    const Time & time()
    {
      return _time;
    }

    const Time & dt()
    {
      return _dt;
    }

  private:

    void _send(OutMessageFlow<mpointer> * outflow, mpointer m)
    {
      assert(m->time >= time());
      assert(m->time <= inflows.nextTime());
      outflow->send(m);
    }
      
    unsigned int _out_flow_index(const string & name)
    {
      for(unsigned int i=0; i<outflows.size(); i++)
	{
	  if(outflows[i].name == name) return i;
	}
      throw  std::domain_error("No outflow "+name+" defined on this node");
    }

    unsigned int _out_flow_sink_index(unsigned int flow, const string & name)
    {
      return outflows[flow].subscriber_index(name);
    }

    unsigned int _in_flow_index(const string & name)
    {
      return inflows.flow_index(name);
    }
    
  public:
    const string name;
  private:
    Time _time;
    Time _dt;
    MessageFlowMultiplexer<mpointer> inflows;
    std::deque<OutMessageFlow<mpointer>> outflows;  
  };
    
}
#endif //DSAAM_NODE_HPP
