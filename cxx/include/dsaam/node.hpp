/***
    node.hpp

***/
#ifndef DSAAM_NODE_HPP
#define DSAAM_NODE_HPP

#include<string>
#include<boost/heap/fibonacci_heap.hpp>
#include<vector>
#include<deque>
#include<list>
#include<functional>
#include<memory>
#include<mutex>
#include<exception>
#include<thread>

#include<dsaam/time.hpp>
#include<dsaam/queue.hpp>
#include<dsaam/exceptions.hpp>

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

  typedef  std::function<void(const std::shared_ptr<MessageBase const>&, const Time &)>
  message_callback_type;

  typedef  std::function<void(const std::shared_ptr<MessageBase const> &)>
  send_callback_type;

  typedef  std::function<void(const Time &)>
  time_callback_type;
  
  static const message_callback_type EMPTY_MESSAGE_CALLBACK;
  static const send_callback_type EMPTY_SEND_CALLBACK;
  static const time_callback_type EMPTY_TIME_CALLBACK;
  
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

  template<class T>
  class MessageQueue : public Queue<T>
  {
  public:
    MessageQueue(Time start_time, Time dt, unsigned int max_size)
      : Queue<T>(max_size), nextTime(start_time), dt(dt) {}

    T && pop()
    {
      T && m = Queue<T>::pop();
      logic_assert(m->time == nextTime,
		   to_string("Time contract breached : Invalid message time expected ",
			     nextTime, " got ", m->time));
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
    MessageFlowMultiplexer(std::vector<InFlow> &inflows, const Time &time, unsigned int max_qsize):
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
      for(size_t i=0; i<inflows.size(); i++)
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
  
    void push(unsigned int flow_index, const T & message)
    {
      T m = message;
      queues[flow_index].push(std::move(m));
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

    void set_flow_callbacks(unsigned int fidx,
			    const ::dsaam::message_callback_type &m_cb,
			    const ::dsaam::time_callback_type &t_cb)
    {
      inflows[fidx].callback = m_cb;
      inflows[fidx].time_callback = t_cb;

    }
    
  public:
    std::vector<InFlow> inflows;
    Time time;
    unsigned int max_qsize;
    MQAllocType mqalloc;
    heap_type heap;
    MessageQueue<T> * queues;
   
    
  };

  template<class T>
  class OutMessageFlow
  {
    typedef boost::heap::fibonacci_heap<unsigned int> heap_type;
  public:
    OutMessageFlow(string name, Time start_time, Time dt, std::vector<Sink> &sinks,
		   unsigned int max_qsize)
      :  name(name), max_qsize(max_qsize), next_time(start_time), dt(dt), sinks(sinks),
	 heap(), heap_handles()
    {
      for(auto s : sinks)
	{
	  typename heap_type::handle_type h = heap.push(max_qsize);
	  heap_handles.push_back(h);
	}
    }

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


    void time_callback(unsigned int subscriber, const Time &)
    {
      unsigned int top = max_qsize;
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
    
    void send(const T & message)
    {
      logic_assert(message->time == next_time,
         to_string("Time contract breached : Invalid message time expected ",
         time, " got ", message->time));
    
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

      next_time  = next_time + dt;
   }

      void set_sink_callback(unsigned int sink_idx, const ::dsaam::send_callback_type & cb)
    {
    sinks[sink_idx].send_callback = cb;
  }

  public:
    const string name;

  private:
    unsigned int max_qsize;
    Time next_time;
    Time dt;
    std::vector<Sink> sinks;
    heap_type heap;
    std::vector<heap_type::handle_type> heap_handles;
    std::mutex m;
    std::condition_variable cv;
  };

  class Node
  {
  public:
    typedef std::shared_ptr<class MessageBase const > mpointer;
    typedef std::function<void(const mpointer &)> push_callback_type;

    Node(string &name, Time &time, Time &dt, std::vector<InFlow> &inflows,
	 std::vector<OutFlow> &outflows, unsigned int max_qsize) :
      name(name), _time(time), _dt(dt),
      inflows(inflows, time, max_qsize), outflows()
    {
      std::cout << "Node::Node(" << this << ", name=" << name << ")" << std::endl;
      for(OutFlow flow : outflows)
	{
	  this->outflows.emplace_back(flow.name, _time, flow.dt,
				      flow.sinks, max_qsize);
	}
    }

    ~Node(){std::cout << "Node::~Node(" << this << ", name=" << name << ")" << std::endl;}

    void next()
    {
      this->inflows.next();
    }

    ::dsaam::send_callback_type send_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&Node::_send, this, &outflows[flow], _1);
    }
    
    ::dsaam::send_callback_type send_callback(const string & flow)
    {
      return send_callback(_out_flow_index(flow));
    }

    ::dsaam::time_callback_type time_callback(unsigned int flow, unsigned int sink)
    {
      using std::placeholders::_1;
      return std::bind(&OutMessageFlow<mpointer>::time_callback, &outflows[flow], sink, _1);
    }

    ::dsaam::time_callback_type time_callback(const string &flow, const string &sink)
    {
      unsigned int iflow = _out_flow_index(flow);
      return time_callback(iflow, _out_flow_sink_index(iflow, sink));
    }
    push_callback_type push_callback(unsigned int flow)
    {
      using std::placeholders::_1;
      return std::bind(&MessageFlowMultiplexer<mpointer>::push, &inflows, flow, _1);
    }

    push_callback_type push_callback(const string &flow)
    {
      return push_callback(_in_flow_index(flow));
    }

    void stepTime(const Time & t)
    {
      logic_assert(t >= _time,
		   to_string("Time contract breached, stepping back in time from ",_time," to ",t));
      _time = t;
    }

    const Time & time()
    {
      return _time;
    }

    const Time & nextAt()
    {
      return std::move(inflows.nextTime());
    }

    const Time & dt()
    {
      return _dt;
    }

    void set_inflow_callbacks(const string & ifname,
			      const ::dsaam::message_callback_type &m_cb,
			      const ::dsaam::time_callback_type &t_cb)
    {
      inflows.set_flow_callbacks(_in_flow_index(ifname), m_cb, t_cb);
    }

    void set_outflow_callback(const string & ofname,
			      const string & sinkname,
			      const ::dsaam::send_callback_type &cb)
    {
      unsigned int odx = _out_flow_index(ofname);
      outflows[odx].set_sink_callback(_out_flow_sink_index(odx, sinkname), cb);
    }

  private:

    void _send(OutMessageFlow<mpointer> * outflow, const mpointer & m)
    {
      logic_assert(m->time >= time(),
		   to_string("Time contract breached : sending message with time ", m->time,
			     " in the future  (current time is ", time()));
      logic_assert(m->time <= inflows.nextTime(),
		   to_string("Time contract breached: sending message at ",
			     m->time, " before arrival of next message at ", inflows.nextTime()));
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

  class OneThreadNode : public Node
{
public:
  OneThreadNode(string &name, Time &time, Time &dt, std::vector<InFlow> &inflows,
	 std::vector<OutFlow> &outflows, unsigned int max_qsize)
    : Node(name,time,dt,inflows,outflows,max_qsize), stopped(false),
      _thread() {} 


  void start()
  {
    if(!_thread.joinable()) _thread = std::thread(&OneThreadNode::run, this);
  }
  
  void stop()
  {
    stopped = true;
  }

  void join()
  {
    _thread.join();
  }

  virtual void init() = 0;

  virtual void step(const Time &) = 0;

private:
  void run()
  {
    init();
    Time t = time() + dt();
    while (!stopped)
      {
	next();
	while(t <= nextAt()) { step(t); stepTime(t); t = t + dt(); }
      }
  }

private:
  bool stopped;
  std::thread _thread;
};

}
#endif //DSAAM_NODE_HPP
