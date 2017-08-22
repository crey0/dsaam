#ifndef DSAAM_INFLOWS_HPP
#define DSAAM_INFLOWS_HPP

#include<dsaam/common.hpp>
#include<dsaam/queue.hpp>
#include<dsaam/binary_heap.hpp>

namespace dsaam
{
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
      nextTime = nextTime + dt;
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
  inline bool operator<(const MessageQueue<T> &l, const MessageQueue<T> &r)
  { return l.nextAt() < r.nextAt();}

  template<typename T>
  inline bool operator>(const MessageQueue<T> &l, const MessageQueue<T> &r)
  { return r < l;}
  
  template<class T>
  class MessageFlowMultiplexer
  {
       
    class heap_data;
    typedef  binary_heap<heap_data, std::greater<heap_data>> heap_type;
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
	  (*h).value.handle = h;
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

    unsigned int flow_index(const string & name) const
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
      //std::cout << to_string("[",std::this_thread::get_id(),"] pushing on flow ",flow_index,
      //			     "/",&queues[flow_index], "\n");
      queues[flow_index].push(std::move(m));
    }

    void next()
    {
      heap_data & q = const_cast<heap_data &>(heap.top());
      //std::cout << to_string("[",std::this_thread::get_id(),"] [",time,"] popping on queue ",q.flow.name,"\n");
      auto m = q.queue.pop();
      heap.decrease(q.handle);
      q.flow.time_callback(nextTime());
      q.flow.callback(std::move(m), nextTime());
    }

    const Time & nextTime() const
    {
      auto & q = heap.top();
      //std::cout << to_string("[",time,"] next message InFlow ",q.flow.name, " at ", q.queue.nextAt(), "\n");
   
      return q.queue.nextAt();
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
}

#endif
