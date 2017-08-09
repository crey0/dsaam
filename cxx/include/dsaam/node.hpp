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
  
  typedef struct InFlow
  {
    InFlow(string name, Time dt,
	   message_callback &callback, message_callback &time_callback)
      : name(name), dt(dt), callback(callback), time_callback(time_callback) {}
    string name;
    Time dt;
    message_callback &callback;
    message_callback &time_callback;
  } InFlow;
  
  typedef struct Sink
  {
    Sink(string name, message_callback &send_callback)
      : name(name), send_callback(send_callback) {}
    string name;
    message_callback &send_callback;
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
  
  void push(unsigned int flow_index, T && message)
  {
    queues[flow_index].push(std::move(message));
  }

  void next()
  {
    heap_data & q = const_cast<heap_data &>(heap.top());
    auto m = q.queue.pop();
    heap.update(q.handle);
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
  
    void time_callback(unsigned int subscriber, const Time &t)
    {
      int top = max_qsize;
      {
	std::lock_guard<std::mutex>(this->m);
      
	typename heap_type::handle_type h = heap_handles[subscriber]
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
      name(name), time(time), dt(dt),
      inflows(inflows, time, max_qsize), outflows()
    {
      for(OutFlow flow : outflows)
	{
	  this->outflows.emplace_back(flow.name, time, flow.dt,
						  flow.sinks, max_qsize);
	}
    }

    void next()
    {
	this->inflows.next();
    }

    void send_callback(mpointer mp)
    {
      assert(mp->time > time);
    }
  public:
    string name;
    Time time;
    Time dt;
    MessageFlowMultiplexer<mpointer> inflows;
    std::deque<OutMessageFlow<mpointer>> outflows;  
  };
    
}
#endif //DSAAM_NODE_HPP
