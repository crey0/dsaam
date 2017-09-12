#ifndef DSAAM_INFLOWS_HPP
#define DSAAM_INFLOWS_HPP

#include<dsaam/queue.hpp>
#include<dsaam/binary_heap.hpp>
#include<deque>

namespace dsaam
{
  template<class M, class T, class FMT>
  class MessageQueue : public Queue<M>
  {
  public:
    MessageQueue(const T &start_time,const T &dt, unsigned int max_size)
      : Queue<M>(max_size), nextTime(start_time), dt(dt) {}

    M && pop()
    {
      M && m = Queue<M>::pop();
      logic_assert(FMT::time(m) == nextTime,
		   to_string("Time contract breached : Invalid message time expected ",
			     nextTime, " got ", FMT::time(m)));
      nextTime = nextTime + dt;
      return std::move(m);
    }

    const T & nextAt() const
    {
      return nextTime; 
    }
      
  private:
    T nextTime;
    T dt;
  };

  template<typename M, typename T, typename FMT>
  inline bool operator<(const MessageQueue<M, T, FMT> &l, const MessageQueue<M, T, FMT> &r)
  { return l.nextAt() < r.nextAt();}

  template<typename M, typename T, typename FMT>
  inline bool operator>(const MessageQueue<M, T, FMT> &l, const MessageQueue<M, T, FMT> &r)
  { return r < l;}
  
  template<class M, class T, class FMT, class I>
  class MessageFlowMultiplexer
  {
    using InFlow = I;
    class heap_data;
    typedef  binary_heap<heap_data, std::greater<heap_data>> heap_type;
    
    class heap_data
    {
    public:
      heap_data(InFlow &flow, MessageQueue<M, T, FMT> & queue) : flow(flow), queue(queue) {}
      InFlow &flow;
      MessageQueue<M, T, FMT> & queue;
      typename heap_type::handle_type handle;

      bool operator>(const heap_data& r) const { return queue > r.queue;}
      bool operator<(const heap_data& r) const { return queue > r.queue;}
    };


  public:
    MessageFlowMultiplexer(const T &time, unsigned int default_qsize):
      inflows(), time(time), default_qsize(default_qsize), heap(), queues()
    {}

    void setup_inflow(InFlow flow)
    {
      auto p = std::unique_ptr<MessageQueue<M, T, FMT>>(
		    new MessageQueue<M, T, FMT>(time, flow.dt,
						flow.qsize>0?flow.qsize:default_qsize));
      queues.push_back(std::move(p));
      inflows.push_back(flow);
      
      size_t index = queues.size()-1;
      typename heap_type::handle_type h = heap.push(heap_data(inflows.back(),
						    *this->queues[index]));
      (*h).value.handle = h;
      
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
  
    void push(unsigned int flow_index, const M & message)
    {
      M m = message;
      //std::cout << to_string("[",std::this_thread::get_id(),"] pushing on flow ",flow_index,
      //			     "/",&queues[flow_index], "\n");
      queues[flow_index]->push(std::move(m));
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

    const T & nextTime() const
    {
      auto & q = heap.top();
      //std::cout << to_string("[",time,"] next message InFlow ",q.flow.name, " at ", q.queue.nextAt(), "\n");
   
      return q.queue.nextAt();
    }
    
  public:
    std::deque<InFlow> inflows;
    T time;
    unsigned int default_qsize;
    heap_type heap;
    std::vector<std::unique_ptr<MessageQueue<M, T, FMT>>> queues;
   
    
  };
}

#endif
